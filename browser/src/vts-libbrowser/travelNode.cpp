/**
 * Copyright (c) 2017 Melown Technologies SE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * *  Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "map.hpp"

namespace vts
{

static const vec3 Nan3 = {
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN()
};

static const vec3 Inf3 = {
    std::numeric_limits<double>::infinity(),
    std::numeric_limits<double>::infinity(),
    std::numeric_limits<double>::infinity()
};

void MapImpl::Traveler::clear()
{
    root.reset();
    loadMetaQueue.clear();
    loadDrawsQueue.clear();
    drawQueue.clear();
}

class TravelNode : public std::enable_shared_from_this<TravelNode>
{
public:
    // hierarchy
    std::array<std::shared_ptr<TravelNode>, 4> childs;
    std::array<bool, 4> childsAvailable;
    const NodeInfo nodeInfo;
    TravelNode *const parent;

    // meta
    std::vector<std::shared_ptr<MetaTile>> metaTiles;
    boost::optional<vtslibs::vts::MetaNode> meta;
    const MapConfig::SurfaceStackItem *surface;

    // visibility
    struct Obb
    {
        std::array<vec3, 2> points;
        mat4 rotInv;
        Obb()
        {
            points.fill(Nan3);
        }
    };
    boost::optional<Obb> obb;
    std::array<vec3, 8> cornersPhys;
    std::array<vec3, 2> aabbPhys;
    vec3 surrogatePhys;

    // renders
    struct Renders
    {
        std::vector<RenderTask> grid, opaque, transparent;
        std::vector<vtslibs::registry::CreditId> credits;

        void clear()
        {
            grid.clear();
            opaque.clear();
            transparent.clear();
            credits.clear();
        }

        bool ready() const
        {
            for (auto &it : opaque)
                if (!it.ready())
                    return false;
            for (auto &it : transparent)
                if (!it.ready())
                    return false;
            return true;
        }

        bool empty() const
        {
            return opaque.empty() && transparent.empty();
        }
    } renders;

    // loading draws
    struct LoadingDraws
    {
        std::shared_ptr<MeshAggregate> meshAgg;
        // todo
    };
    std::shared_ptr<LoadingDraws> loadingDraws;

    // travel
    struct Travel
    {
        struct LodRange
        {
            uint16 a, b;
            void update(uint16 c)
            {
                a = std::min(a, c);
                b = std::max(b, c);
            }
            void update(const LodRange &c)
            {
                a = std::min(a, c.a);
                b = std::max(b, c.b);
            }
        } optimal;

        Travel() : optimal{65535, 0}
        {}
    };
    boost::optional<Travel> travel;

    TravelNode(TravelNode *const parent, const NodeInfo &info)
        : childsAvailable{false, false, false, false},
          nodeInfo(info), parent(parent), surface(nullptr)
    {
        cornersPhys.fill(Nan3);
        aabbPhys.fill(Nan3);
        surrogatePhys = Nan3;
    }
};

namespace
{

void sortAndUnique(std::vector<TravelNode*> &a)
{
    std::sort(a.begin(), a.end());
    a.erase(std::unique(a.begin(), a.end()), a.end());
}

bool aabbTest(const std::array<vec3, 2> &aabb,
              const std::array<vec4, 6> &planes)
{
    assert(aabb[0] == aabb[0]);
    assert(planes[0] == planes[0]);
    for (uint32 i = 0; i < 6; i++)
    {
        const vec4 &p = planes[i]; // current plane
        vec3 pv = vec3( // current p-vertex
                aabb[!!(p[0] > 0)](0),
                aabb[!!(p[1] > 0)](1),
                aabb[!!(p[2] > 0)](2));
        double d = dot(vec4to3(p), pv);
        if (d < -p[3])
            return false;
    }
    return true;
}

vec4 column(const mat4 &m, uint32 index)
{
    return vec4(m(index, 0), m(index, 1), m(index, 2), m(index, 3));
}

void frustumPlanes(const mat4 &vp, std::array<vec4, 6> &planes)
{
    vec4 c0 = column(vp, 0);
    vec4 c1 = column(vp, 1);
    vec4 c2 = column(vp, 2);
    vec4 c3 = column(vp, 3);
    planes[0] = c3 + c0;
    planes[1] = c3 - c0;
    planes[2] = c3 + c1;
    planes[3] = c3 - c1;
    planes[4] = c3 + c2;
    planes[5] = c3 - c2;
}

vec3 lowerUpperCombine(uint32 i)
{
    vec3 res;
    res(0) = (i >> 0) % 2;
    res(1) = (i >> 1) % 2;
    res(2) = (i >> 2) % 2;
    return res;
}

void updateRangeToHalf(float &a, float &b, int which)
{
    a *= 0.5;
    b *= 0.5;
    if (which)
    {
        a += 0.5;
        b += 0.5;
    }
}

} // namespace

bool MapImpl::visibilityTest(TravelNode *trav)
{
    assert(trav->meta);

    // aabb test
    if (!aabbTest(trav->aabbPhys, renderer.frustumPlanes))
        return false;

    // additional obb test
    if (trav->obb)
    {
        TravelNode::Obb &obb = *trav->obb;
        std::array<vec4, 6> planes;
        frustumPlanes(renderer.viewProj * obb.rotInv, planes);
        if (!aabbTest(obb.points, planes))
            return false;
    }

    // all tests passed
    return true;
}

bool MapImpl::coarsenessTest(TravelNode *trav)
{
    assert(trav->meta);

    bool applyTexelSize = trav->meta->flags()
            & vtslibs::vts::MetaNode::Flag::applyTexelSize;
    bool applyDisplaySize = trav->meta->flags()
            & vtslibs::vts::MetaNode::Flag::applyDisplaySize;

    if (!applyTexelSize && !applyDisplaySize)
        return false;

    double result = 0;

    if (applyTexelSize)
    {
        assert(trav->meta->texelSize == trav->meta->texelSize);
        assert(trav->cornersPhys[0] == trav->cornersPhys[0]);
        vec3 up = renderer.perpendicularUnitVector * trav->meta->texelSize;
        for (const vec3 &c : trav->cornersPhys)
        {
            vec3 c1 = c - up * 0.5;
            vec3 c2 = c1 + up;
            c1 = vec4to3(renderer.viewProj * vec3to4(c1, 1), true);
            c2 = vec4to3(renderer.viewProj * vec3to4(c2, 1), true);
            double len = std::abs(c2[1] - c1[1]) * renderer.windowHeight * 0.5;
            result = std::max(result, len);
        }
    }

    if (applyDisplaySize)
    {
        // todo
    }

    return result < options.maxTexelToPixelScale;
}

void MapImpl::travelUpdateMeta(TravelNode *trav)
{
    assert(!trav->meta);
    assert(!trav->surface);
    assert(!trav->parent || trav->parent->meta);
    assert(!trav->loadingDraws);
    assert(trav->renders.empty());

    // statistics
    statistics.currentNodeMetaUpdates++;

    const TileId nodeId = trav->nodeInfo.nodeId();

    // find all metatiles
    std::vector<std::shared_ptr<MetaTile>> &metaTiles
            = trav->metaTiles;
    if (metaTiles.empty())
    {
        metaTiles.resize(mapConfig->surfaceStack.size());
        const UrlTemplate::Vars tileIdVars(roundIdByTileBinaryOrder(nodeId));
        for (uint32 i = 0, e = metaTiles.size(); i != e; i++)
        {
            if (trav->parent)
            {
                const std::shared_ptr<MetaTile> &p
                        = trav->parent->metaTiles[i];
                if (!p)
                    continue;
                TileId pid = vtslibs::vts::parent(nodeId);
                uint32 idx = vtslibs::vts::child(nodeId);
                const vtslibs::vts::MetaNode &node = p->get(pid);
                if ((node.flags()
                     & (vtslibs::vts::MetaNode::Flag::ulChild << idx)) == 0)
                    continue;
            }
            metaTiles[i] = getMetaTile(mapConfig->surfaceStack[i].surface
                                 ->urlMeta(tileIdVars));
        }
    }
    bool determined = true;
    for (auto &m : metaTiles)
    {
        if (!m)
            continue;
        m->updatePriority(1); // todo
        touchResource(m);
        switch (getResourceValidity(m))
        {
        case Validity::Indeterminate:
            determined = false;
            break;
        case Validity::Invalid:
            m.reset();
            continue;
        case Validity::Valid:
            break;
        }
    }
    if (!determined)
        return;

    // find topmost nonempty surface
    MapConfig::SurfaceStackItem *topmost = nullptr;
    const vtslibs::vts::MetaNode *node = nullptr;
    std::array<bool, 4> &childsAvailable = trav->childsAvailable;
    for (uint32 i = 0, e = metaTiles.size(); i != e; i++)
    {
        if (!metaTiles[i])
            continue;
        const vtslibs::vts::MetaNode &n = metaTiles[i]->get(nodeId);
        for (uint32 i = 0; i < 4; i++)
            childsAvailable[i] = childsAvailable[i]
                    || (n.childFlags()
                        & (vtslibs::vts::MetaNode::Flag::ulChild << i));
        if (topmost || n.alien() != mapConfig->surfaceStack[i].alien)
            continue;
        if (n.geometry())
        {
            node = &n;
            if (renderer.tilesetMapping)
            {
                assert(n.sourceReference > 0 && n.sourceReference
                       <= renderer.tilesetMapping->surfaceStack.size());
                topmost = &renderer.tilesetMapping
                        ->surfaceStack[n.sourceReference];
            }
            else
                topmost = &mapConfig->surfaceStack[i];
        }
        if (!node)
            node = &n;
    }
    if (!node)
        return; // all surfaces failed to download, what can i do?

    // corners
    if (!vtslibs::vts::empty(node->geomExtents)
            && !trav->nodeInfo.srs().empty())
    {
        vec2 fl = vecFromUblas<vec2>(trav->nodeInfo.extents().ll);
        vec2 fu = vecFromUblas<vec2>(trav->nodeInfo.extents().ur);
        vec3 el = vec2to3(fl, node->geomExtents.z.min);
        vec3 eu = vec2to3(fu, node->geomExtents.z.max);
        vec3 ed = eu - el;
        vec3 *corners = trav->cornersPhys.data();
        for (uint32 i = 0; i < 8; i++)
        {
            vec3 f = lowerUpperCombine(i).cwiseProduct(ed) + el;
            f = convertor->convert(f, trav->nodeInfo.node(), Srs::Physical);
            corners[i] = f;
        }

        // obb
        if (trav->nodeInfo.distanceFromRoot() > 4)
        {
            vec3 center = vec3(0,0,0);
            for (uint32 i = 0; i < 8; i++)
                center += corners[i];
            center /= 8;

            vec3 f = corners[4] - corners[0];
            vec3 u = corners[2] - corners[0];
            mat4 t = lookAt(center, center + f, u);

            TravelNode::Obb obb;
            obb.rotInv = t.inverse();
            double di = std::numeric_limits<double>::infinity();
            vec3 vi(di, di, di);
            obb.points[0] = vi;
            obb.points[1] = -vi;

            for (uint32 i = 0; i < 8; i++)
            {
                vec3 p = vec4to3(t * vec3to4(corners[i], 1), false);
                obb.points[0] = min(obb.points[0], p);
                obb.points[1] = max(obb.points[1], p);
            }

            trav->obb = obb;
        }
    }
    else if (node->extents.ll != node->extents.ur)
    {
        vec3 fl = vecFromUblas<vec3>(node->extents.ll);
        vec3 fu = vecFromUblas<vec3>(node->extents.ur);
        vec3 fd = fu - fl;
        vec3 el = vecFromUblas<vec3>
                (mapConfig->referenceFrame.division.extents.ll);
        vec3 eu = vecFromUblas<vec3>
                (mapConfig->referenceFrame.division.extents.ur);
        vec3 ed = eu - el;
        for (uint32 i = 0; i < 8; i++)
        {
            vec3 f = lowerUpperCombine(i).cwiseProduct(fd) + fl;
            trav->cornersPhys[i] = f.cwiseProduct(ed) + el;
        }
    }

    // aabb
    if (trav->nodeInfo.distanceFromRoot() > 2)
    {
        assert(trav->cornersPhys[0] == trav->cornersPhys[0]);
        trav->aabbPhys[0]
                = trav->aabbPhys[1]
                = trav->cornersPhys[0];
        for (const vec3 &it : trav->cornersPhys)
        {
            trav->aabbPhys[0] = min(trav->aabbPhys[0], it);
            trav->aabbPhys[1] = max(trav->aabbPhys[1], it);
        }
    }
    else
    {
        trav->aabbPhys[0] = -Inf3;
        trav->aabbPhys[1] = Inf3;
    }

    // surrogate
    if (vtslibs::vts::GeomExtents::validSurrogate(
                node->geomExtents.surrogate))
    {
        vec2 exU = vecFromUblas<vec2>(trav->nodeInfo.extents().ur);
        vec2 exL = vecFromUblas<vec2>(trav->nodeInfo.extents().ll);
        vec3 sds = vec2to3((exU + exL) * 0.5,
                           node->geomExtents.surrogate);
        trav->surrogatePhys = convertor->convert(sds,
                            trav->nodeInfo.node(), Srs::Physical);
    }

    trav->meta = *node;
    trav->surface = topmost;

    // update priority
    //trav->priority = computeResourcePriority(trav);

    // prefetch internal textures
    if (node->geometry())
    {
        assert(trav->surface);
        for (uint32 i = 0; i < node->internalTextureCount(); i++)
            preloadInternalTexture(trav, i);
    }

    // is coarsest lod with data?
    /*
    trav->coarsestWithData = !!trav->meta->surface
            && (!trav->parent || !trav->parent->meta->surface);
    */
}

std::shared_ptr<Resource> MapImpl::preloadInternalTexture(TravelNode *trav,
                                                       uint32 subMeshIndex)
{
    assert(trav->meta);
    assert(trav->surface);
    UrlTemplate::Vars vars(trav->nodeInfo.nodeId(),
            vtslibs::vts::local(trav->nodeInfo), subMeshIndex);
    std::shared_ptr<Resource> res = getTexture(
                trav->surface->surface->urlIntTex(vars));
    res->updatePriority(1); // todo
    return res;
}

void MapImpl::renderNode(TravelNode *trav, const vec4f &uvClip)
{
    assert(trav->meta);
    assert(trav->surface);
    assert(!trav->renders.empty());
    assert(!trav->loadingDraws);
    assert(trav->renders.ready());

    // statistics
    statistics.meshesRenderedTotal++;
    statistics.meshesRenderedPerLod[std::min<uint32>(
        trav->nodeInfo.nodeId().lod, MapStatistics::MaxLods - 1)]++;

    // opaque meshes
    if (options.debugRenderOpaqueMeshes)
    {
        for (const RenderTask &r : trav->renders.opaque)
            draws.opaque.emplace_back(r, uvClip.data(), this);
    }

    // transparent meshes
    if (options.debugRenderTransparentMeshes)
    {
        for (const RenderTask &r : trav->renders.transparent)
            draws.transparent.emplace_back(r, uvClip.data(), this);
    }

    // mesh box
    if (options.debugRenderMeshBoxes)
    {
        RenderTask task;
        task.mesh = getMeshRenderable(
                    "internal://data/meshes/aabb.obj");
        task.mesh->priority = std::numeric_limits<float>::infinity();
        if (task.ready())
        {
            for (RenderTask &r : trav->renders.opaque)
            {
                task.model = r.model;
                if (trav->surface)
                    task.color = vec3to4f(trav->surface->color, task.color(3));
                draws.Infographic.emplace_back(task, this);
            }
        }
    }

    // credits
    for (auto &it : trav->renders.credits)
        renderer.credits.hit(Credits::Scope::Imagery, it,
                             trav->nodeInfo.distanceFromRoot());
}

void MapImpl::renderNodePartial(TravelNode *trav, vec4f uvClip)
{
    if (!trav->parent || !trav->parent->surface)
        return;

    auto id = trav->nodeInfo.nodeId();
    float *arr = uvClip.data();
    updateRangeToHalf(arr[0], arr[2], id.x % 2);
    updateRangeToHalf(arr[1], arr[3], 1 - (id.y % 2));

    if (!trav->parent->renders.empty() && trav->parent->renders.ready())
        renderNode(trav->parent, uvClip);
    else
        renderNodePartial(trav->parent, uvClip);
}

bool MapImpl::travDetermineDrawsOld(TravelNode *trav)
{
    assert(trav->meta);
    assert(trav->surface);
    assert(trav->renders.empty());

    // statistics
    //statistics.currentNodeDrawsUpdates++;

    // update priority
    //trav->priority = computeResourcePriority(trav);

    const TileId nodeId = trav->nodeInfo.nodeId();

    // aggregate mesh
    std::string meshAggName = trav->surface->surface->urlMesh(
            UrlTemplate::Vars(nodeId, vtslibs::vts::local(trav->nodeInfo)));
    std::shared_ptr<MeshAggregate> meshAgg = getMeshAggregate(meshAggName);
    meshAgg->updatePriority(1); // todo
    switch (getResourceValidity(meshAggName))
    {
    case Validity::Invalid:
        trav->surface = nullptr;
        // no break here
    case Validity::Indeterminate:
        return false;
    case Validity::Valid:
        break;
    }

    bool determined = true;
    std::vector<RenderTask> newOpaque;
    std::vector<RenderTask> newTransparent;
    std::vector<vtslibs::registry::CreditId> newCredits;

    // iterate over all submeshes
    for (uint32 subMeshIndex = 0, e = meshAgg->submeshes.size();
         subMeshIndex != e; subMeshIndex++)
    {
        const MeshPart &part = meshAgg->submeshes[subMeshIndex];
        std::shared_ptr<GpuMesh> mesh = part.renderable;

        // external bound textures
        if (part.externalUv)
        {
            std::string surfaceName;
            if (trav->surface->surface->name.size() > 1)
                surfaceName = trav->surface->surface
                        ->name[part.surfaceReference - 1];
            else
                surfaceName = trav->surface->surface->name.back();
            const vtslibs::registry::View::BoundLayerParams::list &boundList
                    = mapConfig->view.surfaces[surfaceName];
            BoundParamInfo::List bls(boundList.begin(), boundList.end());
            if (part.textureLayer)
            {
                bls.push_back(BoundParamInfo(
                        vtslibs::registry::View::BoundLayerParams(
                        mapConfig->boundLayers.get(part.textureLayer).id)));
            }
            switch (reorderBoundLayers(trav->nodeInfo, subMeshIndex,
                                       bls, 1)) // todo
            {
            case Validity::Indeterminate:
                determined = false;
                // no break here
            case Validity::Invalid:
                continue;
            case Validity::Valid:
                break;
            }
            bool allTransparent = true;
            for (BoundParamInfo &b : bls)
            {
                // credits
                {
                    MapConfig::BoundInfo *l = b.bound;
                    assert(l);
                    for (auto &it : l->credits)
                    {
                        auto c = renderer.credits.find(it.first);
                        if (c)
                            newCredits.push_back(*c);
                    }
                }

                // draw task
                RenderTask task;
                task.textureColor = getTexture(b.bound->urlExtTex(b.vars));
                task.textureColor->updatePriority(1); // todo
                task.textureColor->availTest = b.bound->availability;
                switch (getResourceValidity(task.textureColor))
                {
                case Validity::Indeterminate:
                    determined = false;
                    // no break here
                case Validity::Invalid:
                    continue;
                case Validity::Valid:
                    break;
                }
                if (!b.watertight)
                {
                    task.textureMask = getTexture(b.bound->urlMask(b.vars));
                    task.textureMask->updatePriority(1); // todo
                    switch (getResourceValidity(task.textureMask))
                    {
                    case Validity::Indeterminate:
                        determined = false;
                        // no break here
                    case Validity::Invalid:
                        continue;
                    case Validity::Valid:
                        break;
                    }
                }
                task.color(3) = b.alpha ? *b.alpha : 1;
                task.meshAgg = meshAgg;
                task.mesh = mesh;
                task.model = part.normToPhys;
                task.uvm = b.uvMatrix();
                task.externalUv = true;
                if (b.transparent)
                    newTransparent.push_back(task);
                else
                    newOpaque.push_back(task);
                allTransparent = allTransparent && b.transparent;
            }
            if (!allTransparent)
                continue;
        }

        // internal texture
        if (part.internalUv)
        {
            RenderTask task;
            task.textureColor = preloadInternalTexture(trav, subMeshIndex);
            switch (getResourceValidity(task.textureColor))
            {
            case Validity::Indeterminate:
                determined = false;
                // no break here
            case Validity::Invalid:
                continue;
            case Validity::Valid:
                break;
            }
            task.meshAgg = meshAgg;
            task.mesh = mesh;
            task.model = part.normToPhys;
            task.uvm = identityMatrix3().cast<float>();
            task.externalUv = false;
            newOpaque.insert(newOpaque.begin(), task);
        }
    }

    if (determined)
    {
        assert(trav->renders.empty());
        std::swap(trav->renders.opaque, newOpaque);
        std::swap(trav->renders.transparent, newTransparent);
        if (trav->renders.empty())
        {
            trav->surface = nullptr;
            return true;
        }
        for (auto it : trav->meta->credits())
            newCredits.push_back(it);
        std::swap(trav->renders.credits, newCredits);
    }

    return determined;
}

void MapImpl::travelUpdateDraws(TravelNode *trav)
{
    assert(trav->meta);
    assert(trav->surface);
    assert(trav->renders.empty());
    assert(trav->loadingDraws);

    // statistics
    statistics.currentNodeDrawsUpdates++;

    bool res = travDetermineDrawsOld(trav);
    if (res)
    {
        trav->loadingDraws.reset();
        if (trav->surface)
        {
            assert(!trav->renders.empty());
        }
        else
        {
            assert(trav->renders.empty());
        }
    }
    else
    {
        assert(trav->renders.empty());
    }



    /*
    const TileId nodeId = trav->nodeInfo.nodeId();

    // aggregate mesh
    std::shared_ptr<MeshAggregate> &meshAgg = trav->loadingDraws->meshAgg;
    if (!meshAgg)
    {
        std::string meshAggName = trav->surface->surface->urlMesh(
            UrlTemplate::Vars(nodeId, vtslibs::vts::local(trav->nodeInfo)));
        meshAgg = getMeshAggregate(meshAggName);
    }
    meshAgg->updatePriority(1); // todo
    switch (getResourceValidity(meshAgg))
    {
    case Validity::Invalid:
        trav->surface = nullptr;
        return;
    case Validity::Indeterminate:
        return;
    case Validity::Valid:
        break;
    }
    */
}

void MapImpl::travelClearingNode(TravelNode *trav)
{
    if (!trav->travel)
    {
        for (auto &it : trav->childs)
            it.reset();
        trav->renders.clear();
        trav->loadingDraws.reset();
    }
    else
    {
        for (auto &it: trav->childs)
            if (it)
                travelClearingNode(it.get());
    }
}

void MapImpl::travelDetermineHierarchy(TravelNode *trav)
{
    // statistics
    statistics.metaNodesTraversedTotal++;
    statistics.metaNodesTraversedPerLod[std::min<uint32>(
        trav->nodeInfo.nodeId().lod, MapStatistics::MaxLods - 1)]++;

    // meta update
    if (!trav->meta)
    {
        travel.loadMetaQueue.push_back(trav);
        trav->travel.emplace();
        return;
    }

    // visibility
    if (!visibilityTest(trav))
    {
        trav->travel.reset();
        return;
    }

    // travel data
    trav->travel.emplace();
    uint32 current = trav->nodeInfo.nodeId().lod;

    // childs
    if (!coarsenessTest(trav))
    {
        // we need to go deeper
        auto children = vtslibs::vts::children(trav->nodeInfo.nodeId());
        uint32 i = 0;
        uint32 childsCount = 0;
        bool ready = true;
        for (auto &it : trav->childs)
        {
            if (!it && trav->childsAvailable[i])
            {
                it = std::make_shared<TravelNode>(trav,
                        trav->nodeInfo.child(children[i]));
            }
            if (it)
            {
                childsCount++;
                if (!it->meta)
                {
                    travelDetermineHierarchy(it.get());
                    ready = false;
                }
            }
            i++;
        }
        if (ready && childsCount > 0)
        {
            // all children have meta data loaded
            // descend
            for (auto &it : trav->childs)
            {
                if (it)
                {
                    travelDetermineHierarchy(it.get());
                    if (it->travel)
                        trav->travel->optimal.update(it->travel->optimal);
                }
            }
            return;
        }
    }

    // render this node
    trav->travel->optimal.update(current);
    travel.drawQueue.push_back(trav);
}

void MapImpl::travelDetermineDrawLoads(TravelNode *trav)
{
    if (!trav->meta)
        return;

    if (trav->surface && trav->travel)
    {
        // load all, for now
        if (!trav->loadingDraws && trav->renders.empty())
            trav->loadingDraws = std::make_shared<TravelNode::LoadingDraws>();
        if (trav->loadingDraws)
            travel.loadDrawsQueue.push_back(trav);
    }

    for (const auto &it : trav->childs)
        if (it)
            travelDetermineDrawLoads(it.get());
}

void MapImpl::travelDetermineRenders(TravelNode *trav)
{
    if (!trav->renders.empty() && trav->renders.ready())
        renderNode(trav);
    else
        renderNodePartial(trav, vec4f(0,0,1,1));

    // surrogate
    if (options.debugRenderSurrogates)
    {
        RenderTask task;
        task.mesh = getMeshRenderable(
                    "internal://data/meshes/sphere.obj");
        task.mesh->priority = std::numeric_limits<float>::infinity();
        if (task.ready())
        {
            task.model = translationMatrix(trav->surrogatePhys)
                    * scaleMatrix(trav->nodeInfo.extents().size() * 0.03);
            if (trav->surface)
                task.color = vec3to4f(trav->surface->color, task.color(3));
            draws.Infographic.emplace_back(task, this);
        }
    }

    // tile box
    if (options.debugRenderTileBoxes)
    {
        RenderTask task;
        task.mesh = getMeshRenderable(
                    "internal://data/meshes/line.obj");
        task.mesh->priority = std::numeric_limits<float>::infinity();
        if (task.ready())
        {
            //task.color = vec4f(1, 0, 0, 1);
            static const uint32 cora[] = {
                0, 0, 1, 2, 4, 4, 5, 6, 0, 1, 2, 3
            };
            static const uint32 corb[] = {
                1, 2, 3, 3, 5, 6, 7, 7, 4, 5, 6, 7
            };
            for (uint32 i = 0; i < 12; i++)
            {
                vec3 a = trav->cornersPhys[cora[i]];
                vec3 b = trav->cornersPhys[corb[i]];
                task.model = lookAt(a, b);
                draws.Infographic.emplace_back(task, this);
            }
        }
    }
}

void MapImpl::travelTickPrepare()
{
    assert(travel.root);
    sortAndUnique(travel.loadMetaQueue);
    for (const auto &it : travel.loadMetaQueue)
            travelUpdateMeta(it);
    travel.loadMetaQueue.clear();
    sortAndUnique(travel.loadDrawsQueue);
    for (const auto &it : travel.loadDrawsQueue)
        travelUpdateDraws(it);
    travel.loadDrawsQueue.clear();
    travelClearingNode(travel.root.get());
}

void MapImpl::travelTickRender()
{
    assert(travel.root);
    assert(travel.drawQueue.empty());
    travelDetermineHierarchy(travel.root.get());

    travelDetermineDrawLoads(travel.root.get());
    for (auto it : travel.drawQueue)
        travelDetermineRenders(it);
    travel.drawQueue.clear();
}

void MapImpl::initializeTravelRoot()
{
    travel.root = std::make_shared<TravelNode>(nullptr, NodeInfo(
            mapConfig->referenceFrame, TileId(), false, *mapConfig));
}

} // namespace vts
