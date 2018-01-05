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

namespace
{

vec3 lowerUpperCombine(uint32 i)
{
    vec3 res;
    res(0) = (i >> 0) % 2;
    res(1) = (i >> 1) % 2;
    res(2) = (i >> 2) % 2;
    return res;
}

TileId::index_type roundNeighbor1(TileId::index_type c, int x,
                                  TileId::index_type range)
{
    assert(x >= -1 && x <= 1);
    if (c == 0 && x < 0) // index type is unsigned -> avoid overflow
        return range + x;
    return (c + x) % range;
}

TileId roundNeighbor(TileId c, int x, int y)
{
    c.x = roundNeighbor1(c.x, x, 1u << c.lod);
    c.y = roundNeighbor1(c.y, y, 1u << c.lod);
    return c;
}

} // namespace

bool MapImpl::travDetermineMeta(TraverseNode *trav)
{
    assert(!trav->meta);
    assert(trav->childs.empty());
    assert(trav->rendersEmpty());
    assert(!trav->parent || trav->parent->meta);

    // statistics
    statistics.currentNodeMetaUpdates++;

    const TileId nodeId = trav->nodeInfo.nodeId();

    // find all metatiles
    std::vector<std::shared_ptr<MetaTile>> metaTiles;
    metaTiles.resize(mapConfig->surfaceStack.size());
    const UrlTemplate::Vars tileIdVars(roundIdByTileBinaryOrder(nodeId));
    bool determined = true;
    for (uint32 i = 0, e = metaTiles.size(); i != e; i++)
    {
        if (trav->parent)
        {
            const std::shared_ptr<MetaTile> &p
                    = trav->parent->meta->metaTiles[i];
            if (!p)
                continue;
            TileId pid = vtslibs::vts::parent(nodeId);
            uint32 idx = (nodeId.x % 2) + (nodeId.y % 2) * 2;
            const vtslibs::vts::MetaNode &node = p->get(pid);
            if ((node.flags()
                 & (vtslibs::vts::MetaNode::Flag::ulChild << idx)) == 0)
                continue;
        }
        auto m = getMetaTile(mapConfig->surfaceStack[i].surface
                             ->urlMeta(tileIdVars));
        m->updatePriority(trav->priority);
        switch (getResourceValidity(m))
        {
        case Validity::Indeterminate:
            determined = false;
            // no break here
        case Validity::Invalid:
            continue;
        case Validity::Valid:
            break;
        }
        metaTiles[i] = m;
    }
    if (!determined)
        return false;

    // find topmost nonempty surface
    MapConfig::SurfaceStackItem *topmost = nullptr;
    const vtslibs::vts::MetaNode *node = nullptr;
    bool childsAvailable[4] = {false, false, false, false};
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
        return false; // all surfaces failed to download, what can i do?

    trav->meta = *node;
    trav->meta->metaTiles.swap(metaTiles);

    // corners
    if (!vtslibs::vts::empty(node->geomExtents)
            && !trav->nodeInfo.srs().empty())
    {
        vec2 fl = vecFromUblas<vec2>(trav->nodeInfo.extents().ll);
        vec2 fu = vecFromUblas<vec2>(trav->nodeInfo.extents().ur);
        vec3 el = vec2to3(fl, node->geomExtents.z.min);
        vec3 eu = vec2to3(fu, node->geomExtents.z.max);
        vec3 ed = eu - el;
        vec3 *corners = trav->meta->cornersPhys;
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

            TraverseNode::Obb obb;
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

            trav->meta->obb = obb;
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
            trav->meta->cornersPhys[i] = f.cwiseProduct(ed) + el;
        }
    }

    // aabb
    if (trav->nodeInfo.distanceFromRoot() > 2)
    {
        trav->meta->aabbPhys[0]
                = trav->meta->aabbPhys[1]
                = trav->meta->cornersPhys[0];
        for (const vec3 &it : trav->meta->cornersPhys)
        {
            trav->meta->aabbPhys[0] = min(trav->meta->aabbPhys[0], it);
            trav->meta->aabbPhys[1] = max(trav->meta->aabbPhys[1], it);
        }
    }

    // surrogate
    if (vtslibs::vts::GeomExtents::validSurrogate(
                node->geomExtents.surrogate))
    {
        vec2 exU = vecFromUblas<vec2>(trav->nodeInfo.extents().ur);
        vec2 exL = vecFromUblas<vec2>(trav->nodeInfo.extents().ll);
        vec3 sds = vec2to3((exU + exL) * 0.5,
                           node->geomExtents.surrogate);
        trav->meta->surrogatePhys = convertor->convert(sds,
                            trav->nodeInfo.node(), Srs::Physical);
    }

    // surface
    if (topmost)
    {
        trav->meta->surface = topmost;
        // credits
        for (auto it : node->credits())
            trav->meta->credits.push_back(it);
    }

    // prepare children
    vtslibs::vts::Children childs = vtslibs::vts::children(nodeId);
    for (uint32 i = 0; i < 4; i++)
    {
        if (childsAvailable[i])
            trav->childs.push_back(std::make_shared<TraverseNode>(
                        trav, trav->nodeInfo.child(childs[i])));
    }

    // update priority
    trav->priority = computeResourcePriority(trav);

    // prefetch internal textures
    if (node->geometry())
    {
        for (uint32 i = 0; i < node->internalTextureCount(); i++)
            preloadInternalTexture(trav, i);
    }

    // is coarsest lod with data?
    trav->coarsestWithData = !!trav->meta->surface
            && (!trav->parent || !trav->parent->meta->surface);

    return true;
}

bool MapImpl::travDetermineDraws(TraverseNode *trav)
{
    assert(trav->meta);
    assert(trav->meta->surface);
    assert(trav->rendersEmpty());

    // statistics
    statistics.currentNodeDrawsUpdates++;

    // update priority
    //trav->priority = computeResourcePriority(trav);

    const TileId nodeId = trav->nodeInfo.nodeId();

    // aggregate mesh
    std::string meshAggName = trav->meta->surface->surface->urlMesh(
            UrlTemplate::Vars(nodeId, vtslibs::vts::local(trav->nodeInfo)));
    std::shared_ptr<MeshAggregate> meshAgg = getMeshAggregate(meshAggName);
    meshAgg->updatePriority(trav->priority);
    switch (getResourceValidity(meshAggName))
    {
    case Validity::Invalid:
        trav->meta->surface = nullptr;
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
            if (trav->meta->surface->surface->name.size() > 1)
                surfaceName = trav->meta->surface->surface
                        ->name[part.surfaceReference - 1];
            else
                surfaceName = trav->meta->surface->surface->name.back();
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
                                       bls, trav->priority))
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
                task.textureColor->updatePriority(trav->priority);
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
                    task.textureMask->updatePriority(trav->priority);
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
        assert(trav->rendersEmpty());
        std::swap(trav->opaque, newOpaque);
        std::swap(trav->transparent, newTransparent);
        trav->meta->credits.insert(trav->meta->credits.end(),
                             newCredits.begin(), newCredits.end());
        if (trav->rendersEmpty())
            trav->meta->surface = nullptr;
    }

    return determined;
}

bool MapImpl::travInit(TraverseNode *trav, bool skipStatistics)
{
    // statistics
    if (!skipStatistics)
    {
        statistics.metaNodesTraversedTotal++;
        statistics.metaNodesTraversedPerLod[
                std::min<uint32>(trav->nodeInfo.nodeId().lod,
                                 MapStatistics::MaxLods-1)]++;
    }

    // update trav
    trav->lastTimeAccessed = renderer.tickIndex;

    // priority
    trav->priority = trav->meta
            ? computeResourcePriority(trav)
            : trav->parent
              ? trav->parent->priority
              : 0;

    // prepare meta data
    if (!trav->meta)
        return travDetermineMeta(trav);

    return true;
}

void MapImpl::travModeHierarchical(TraverseNode *trav, bool loadOnly)
{
    if (!travInit(trav))
        return;

    touchDraws(trav);
    if (trav->meta->surface && trav->rendersEmpty())
        travDetermineDraws(trav);

    if (loadOnly)
        return;

    if (!visibilityTest(trav))
        return;

    if (coarsenessTest(trav) || trav->childs.empty())
    {
        renderNode(trav);
        return;
    }

    bool ok = true;
    for (auto &t : trav->childs)
    {
        if (!t->meta)
        {
            ok = false;
            continue;
        }
        if (t->meta->surface && t->rendersEmpty())
            ok = false;
    }

    for (auto &t : trav->childs)
        travModeHierarchical(t.get(), !ok);

    if (!ok && !trav->rendersEmpty())
        renderNode(trav);
}

void MapImpl::travModeFlat(TraverseNode *trav)
{
    if (!travInit(trav))
        return;

    if (!visibilityTest(trav))
        return;

    if (coarsenessTest(trav) || trav->childs.empty())
    {
        touchDraws(trav);
        if (trav->meta->surface && trav->rendersEmpty())
            travDetermineDraws(trav);
        if (!trav->rendersEmpty())
            renderNode(trav);
        return;
    }

    for (auto &t : trav->childs)
        travModeFlat(t.get());
}

void MapImpl::travModeBalanced(TraverseNode *trav)
{
    if (!travInit(trav))
        return;

    if (!visibilityTest(trav))
        return;

    double coar = coarsenessValue(trav);
    if (coar < options.maxTexelToPixelScale)
    {
        touchDraws(trav);
        if (trav->meta->surface && trav->rendersEmpty())
            travDetermineDraws(trav);
    }

    bool childsHaveMeta = true;
    for (auto &it : trav->childs)
        childsHaveMeta = childsHaveMeta && travInit(it.get(), true);

    if (coar < options.maxTexelToPixelScale || trav->childs.empty()
            || !childsHaveMeta)
    {
        travBalancedPropagateUp(trav, trav->nodeInfo.nodeId().lod);
        if (!trav->rendersEmpty() && trav->rendersReady())
            renderNode(trav);
        else
            renderNodePartial(trav, trav->nodeInfo.nodeId().lod,
                                       vec4f(0,0,1,1));
        return;
    }

    for (auto &t : trav->childs)
        travModeBalanced(t.get());
}

void MapImpl::travBalancedPropagateUp(TraverseNode *trav,
                                          uint32 originalLod)
{
    uint32 gridlod = options.coarserLodOffset + options.gridsLodOffset;
    uint32 d = originalLod - trav->nodeInfo.nodeId().lod;
    if (d == options.coarserLodOffset
            || (options.enableLoadIntermediateLods
                && d < options.coarserLodOffset)
            || d == gridlod
            || trav->coarsestWithData)
    {
        // preload coarser lod
        touchDraws(trav);
        if (trav->meta->surface && trav->rendersEmpty())
            travDetermineDraws(trav);
    }
    if (d == gridlod && options.enableLoadNeighborGrids)
    {
        // preload neighbors for grid
        TileId id = trav->nodeInfo.nodeId();
        for (int x = -1; x < 2; x++)
            for (int y = -1; y < 2; y++)
                if (x != 0 || y != 0)
                    renderer.nodesToPreload.push_back(roundNeighbor(id, x, y));
    }
    if (d >= gridlod || trav->coarsestWithData)
        return;
    if (trav->parent)
        travBalancedPropagateUp(trav->parent, originalLod);
}

void MapImpl::travPreloadNodes(TraverseNode *trav, TileId target)
{
    TileId id = trav->nodeInfo.nodeId();
    assert(id.lod <= target.lod);

    if (!travInit(trav, true))
        return;

    if (id == target)
    {
        // target found
        touchDraws(trav);
        if (trav->meta->surface && trav->rendersEmpty())
            travDetermineDraws(trav);
    }
    else
    {
        // we need to go deeper
        TileId par = vtslibs::vts::parent(target, target.lod - id.lod - 1);
        for (auto &it : trav->childs)
            if (it->nodeInfo.nodeId() == par)
                return travPreloadNodes(it.get(), target);
    }
}

void MapImpl::traverseRender()
{
    switch (options.traverseMode)
    {
    case TraverseMode::Hierarchical:
        travModeHierarchical(renderer.traverseRoot.get(), false);
        break;
    case TraverseMode::Flat:
        travModeFlat(renderer.traverseRoot.get());
        break;
    case TraverseMode::Balanced:
        travModeBalanced(renderer.traverseRoot.get());
        break;
    }
}

void MapImpl::traversePreloadNodes()
{
    auto &ns = renderer.nodesToPreload;
    std::sort(ns.begin(), ns.end());
    ns.erase(std::unique(ns.begin(), ns.end()), ns.end());
    for (auto it : renderer.nodesToPreload)
        travPreloadNodes(renderer.traverseRoot.get(), it);
    ns.clear();
}

void MapImpl::traverseClearing(TraverseNode *trav)
{
    if (trav->lastTimeAccessed + 5 < renderer.tickIndex)
    {
        trav->clearAll();
        return;
    }
    if (trav->lastTimeTouched + 5 < renderer.tickIndex)
        trav->clearRenders();

    for (auto &it : trav->childs)
        traverseClearing(it.get());
}

} // namespace vts
