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

#include <boost/utility/in_place_factory.hpp>

#include "include/vts-browser/exceptions.hpp"
#include "map.hpp"

namespace vts
{

namespace
{

bool testAndThrow(Resource::State state, const std::string &message)
{
    switch (state)
    {
    case Resource::State::errorRetry:
    case Resource::State::downloaded:
    case Resource::State::downloading:
    case Resource::State::initializing:
        return false;
    case Resource::State::ready:
        return true;
    default:
        LOGTHROW(err4, MapConfigException) << message;
        throw;
    }
}

bool aabbTest(const vec3 aabb[2], const vec4 planes[6])
{
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

void frustumPlanes(const mat4 &vp, vec4 planes[6])
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

MapImpl::Renderer::Renderer() :
    windowWidth(0), windowHeight(0), tickIndex(0)
{}

void MapImpl::renderInitialize()
{
    LOG(info3) << "Render initialize";
}

void MapImpl::renderFinalize()
{
    LOG(info3) << "Render finalize";
}

void MapImpl::setMapConfigPath(const std::string &mapConfigPath,
                               const std::string &authPath,
                               const std::string &sriPath)
{
    LOG(info3) << "Changing map config path to <" << mapConfigPath << ">, "
               << (!authPath.empty() ? "using" : "without")
               << " authentication and "
               << (!sriPath.empty() ?
                       std::string() + "using SRI <" + sriPath + ">"
                     : "without SRI");
    this->mapConfigPath = mapConfigPath;
    resources.authPath = authPath;
    resources.sriPath = sriPath;
    purgeMapConfig();
}

void MapImpl::purgeMapConfig()
{
    LOG(info2) << "Purge map config";

    if (resources.auth)
    {
        resources.auth->state = Resource::State::errorRetry;
        resources.auth->retryTime = 0;
    }
    if (mapConfig)
    {
        mapConfig->state = Resource::State::errorRetry;
        mapConfig->retryTime = 0;
    }

    resources.auth.reset();
    mapConfig.reset();
    renderer.credits.purge();
    resources.searchTasks.clear();
    resetNavigationMode();
    navigation.autoRotation = 0;
    navigation.lastPositionAltitude.reset();
    navigation.positionAltitudeReset.reset();
    body = MapCelestialBody();
    purgeViewCache();
}

void MapImpl::purgeViewCache()
{
    LOG(info2) << "Purge view cache";

    if (mapConfig)
    {
        mapConfig->consolidateView();
        mapConfig->surfaceStack.clear();
    }

    renderer.traverseRoot.reset();
    renderer.tilesetMapping.reset();
    statistics.resetFrame();
    draws = MapDraws();
    credits = MapCredits();
    mapConfigView = "";
    initialized = false;
}

const TileId MapImpl::roundId(TileId nodeId)
{
    uint32 metaTileBinaryOrder = mapConfig->referenceFrame.metaBinaryOrder;
    return TileId (nodeId.lod,
       (nodeId.x >> metaTileBinaryOrder) << metaTileBinaryOrder,
       (nodeId.y >> metaTileBinaryOrder) << metaTileBinaryOrder);
}

Validity MapImpl::reorderBoundLayers(const NodeInfo &nodeInfo,
        uint32 subMeshIndex, BoundParamInfo::List &boundList, double priority)
{
    // prepare all layers
    {
        bool determined = true;
        auto it = boundList.begin();
        while (it != boundList.end())
        {
            switch (it->prepare(nodeInfo, this, subMeshIndex, priority))
            {
            case Validity::Invalid:
                it = boundList.erase(it);
                break;
            case Validity::Indeterminate:
                determined = false;
                // no break here
            case Validity::Valid:
                it++;
            }
        }
        if (!determined)
            return Validity::Indeterminate;
    }

    // skip overlapping layers
    std::reverse(boundList.begin(), boundList.end());
    auto it = boundList.begin(), et = boundList.end();
    while (it != et && (!it->watertight || it->transparent))
        it++;
    if (it != et)
        boundList.erase(++it, et);
    std::reverse(boundList.begin(), boundList.end());

    return Validity::Valid;
}

bool MapImpl::visibilityTest(TraverseNode *trav)
{
    assert(trav->meta);
    // aabb test
    if (!aabbTest(trav->meta->aabbPhys, renderer.frustumPlanes))
        return false;
    // additional obb test
    if (trav->meta->obb)
    {
        TraverseNode::Obb &obb = *trav->meta->obb;
        vec4 planes[6];
        frustumPlanes(renderer.viewProj * obb.rotInv, planes);
        if (!aabbTest(obb.points, planes))
            return false;
    }
    // all tests passed
    return true;
}

bool MapImpl::coarsenessTest(TraverseNode *trav)
{
    assert(trav->meta);
    return coarsenessValue(trav) < options.maxTexelToPixelScale;
}

double MapImpl::coarsenessValue(TraverseNode *trav)
{
    bool applyTexelSize = trav->meta->flags()
            & vtslibs::vts::MetaNode::Flag::applyTexelSize;
    bool applyDisplaySize = trav->meta->flags()
            & vtslibs::vts::MetaNode::Flag::applyDisplaySize;

    if (!applyTexelSize && !applyDisplaySize)
        return std::numeric_limits<double>::infinity();

    double result = 0;

    if (applyTexelSize)
    {
        vec3 up = renderer.perpendicularUnitVector * trav->meta->texelSize;
        for (const vec3 &c : trav->meta->cornersPhys)
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

    return result;
}

void MapImpl::renderNode(TraverseNode *trav, uint32 originalLod,
                         const vec4f &uvClip)
{
    assert(trav->meta);
    assert(!trav->rendersEmpty());
    assert(trav->rendersReady());
    assert(visibilityTest(trav));

    // statistics
    statistics.meshesRenderedTotal++;
    statistics.meshesRenderedPerLod[std::min<uint32>(
        trav->nodeInfo.nodeId().lod, MapStatistics::MaxLods - 1)]++;

    // updates
    trav->lastTimeTouched = renderer.tickIndex;
    uint32 lodDiff = (originalLod == (uint32)-1) ? 0
                : (originalLod - trav->nodeInfo.nodeId().lod);

    // meshes
    if (lodDiff <= options.coarserLodOffset)
    {
        // regular meshes
        if (options.debugRenderOpaqueMeshes)
        {
            for (const RenderTask &r : trav->opaque)
                draws.opaque.emplace_back(r, uvClip.data(), this);
        }
        if (options.debugRenderTransparentMeshes)
        {
            for (const RenderTask &r : trav->transparent)
                draws.transparent.emplace_back(r, uvClip.data(), this);
        }
    }
    else
    {
        // grids
        auto fnc = [&](std::vector<DrawTask> &vec, const RenderTask &r)
        {
            DrawTask d(r, uvClip.data(), this);
            if (d.texColor)
            {
                if (!*resources.gridTexture)
                    return;
                d.texColor = resources.gridTexture->info.userData;
                d.externalUv = true;
                d.color[0] = d.color[1] = d.color[2] = d.color[3] = 1;
                d.flatShading = false;
            }
            vec.push_back(std::move(d));
        };
        if (options.debugRenderOpaqueMeshes)
        {
            for (const RenderTask &r : trav->opaque)
                fnc(draws.opaque, r);
        }
        statistics.meshesRenderedGrids++;
    }

    if (lodDiff == 0)
    {
        // surrogate
        if (options.debugRenderSurrogates)
        {
            RenderTask task;
            task.mesh = getMeshRenderable("internal://data/meshes/sphere.obj");
            task.mesh->priority = std::numeric_limits<float>::infinity();
            task.model = translationMatrix(trav->meta->surrogatePhys)
                    * scaleMatrix(trav->nodeInfo.extents().size() * 0.03);
            if (trav->meta->surface)
                task.color = vec3to4f(trav->meta->surface->color,
                                      task.color(3));
            if (task.ready())
                draws.Infographic.emplace_back(task, this);
        }

        // mesh box
        if (options.debugRenderMeshBoxes)
        {
            for (RenderTask &r : trav->opaque)
            {
                RenderTask task;
                task.model = r.model;
                task.mesh = getMeshRenderable(
                            "internal://data/meshes/aabb.obj");
                task.mesh->priority = std::numeric_limits<float>::infinity();
                task.color = vec4f(0, 0, 1, 1);
                if (task.ready())
                    draws.Infographic.emplace_back(task, this);
            }
        }

        // tile box
        if (options.debugRenderTileBoxes)
        {
            RenderTask task;
            task.mesh = getMeshRenderable("internal://data/meshes/line.obj");
            task.mesh->priority = std::numeric_limits<float>::infinity();
            task.color = vec4f(1, 0, 0, 1);
            if (task.ready())
            {
                static const uint32 cora[] = {
                    0, 0, 1, 2, 4, 4, 5, 6, 0, 1, 2, 3
                };
                static const uint32 corb[] = {
                    1, 2, 3, 3, 5, 6, 7, 7, 4, 5, 6, 7
                };
                for (uint32 i = 0; i < 12; i++)
                {
                    vec3 a = trav->meta->cornersPhys[cora[i]];
                    vec3 b = trav->meta->cornersPhys[corb[i]];
                    task.model = lookAt(a, b);
                    draws.Infographic.emplace_back(task, this);
                }
            }
        }
    }

    // credits
    for (auto &it : trav->meta->credits)
        renderer.credits.hit(Credits::Scope::Imagery, it,
                             trav->nodeInfo.distanceFromRoot());
}

void MapImpl::renderNodePartial(TraverseNode *trav, uint32 originalLod,
                                         vec4f uvClip)
{
    if (!trav->parent || !trav->parent->meta->surface)
        return;

    auto id = trav->nodeInfo.nodeId();
    float *arr = uvClip.data();
    updateRangeToHalf(arr[0], arr[2], id.x % 2);
    updateRangeToHalf(arr[1], arr[3], 1 - (id.y % 2));

    if (!trav->parent->rendersEmpty() && trav->parent->rendersReady())
        renderNode(trav->parent, originalLod, uvClip);
    else
        renderNodePartial(trav->parent, originalLod, uvClip);
}

void MapImpl::touchDraws(const RenderTask &task)
{
    if (task.meshAgg)
        touchResource(task.meshAgg);
    if (task.textureColor)
        touchResource(task.textureColor);
    if (task.textureMask)
        touchResource(task.textureMask);
}

void MapImpl::touchDraws(const std::vector<RenderTask> &renders)
{
    for (auto &it : renders)
        touchDraws(it);
}

void MapImpl::touchDraws(TraverseNode *trav)
{
    trav->lastTimeTouched = renderer.tickIndex;
    for (auto &it : trav->opaque)
        touchDraws(it);
    for (auto &it : trav->transparent)
        touchDraws(it);
}

bool MapImpl::prerequisitesCheck()
{
    if (resources.auth)
    {
        resources.auth->checkTime();
        touchResource(resources.auth);
    }

    if (mapConfig)
        touchResource(mapConfig);

    if (renderer.tilesetMapping)
        touchResource(renderer.tilesetMapping);

    if (initialized)
        return true;

    if (mapConfigPath.empty())
        return false;

    if (!resources.authPath.empty())
    {
        resources.auth = getAuthConfig(resources.authPath);
        if (!testAndThrow(resources.auth->state, "Authentication failure."))
            return false;
    }

    mapConfig = getMapConfig(mapConfigPath);
    if (!testAndThrow(mapConfig->state, "Map config failure."))
        return false;

    // check for virtual surface
    if (!options.debugDisableVirtualSurfaces)
    {
        std::vector<std::string> viewSurfaces;
        viewSurfaces.reserve(mapConfig->view.surfaces.size());
        for (auto &it : mapConfig->view.surfaces)
            viewSurfaces.push_back(it.first);
        std::sort(viewSurfaces.begin(), viewSurfaces.end());
        for (vtslibs::vts::VirtualSurfaceConfig &it :mapConfig->virtualSurfaces)
        {
            std::vector<std::string> virtSurfaces(it.id.begin(), it.id.end());
            if (virtSurfaces.size() != viewSurfaces.size())
                continue;
            std::vector<std::string> virtSurfaces2(virtSurfaces);
            std::sort(virtSurfaces2.begin(), virtSurfaces2.end());
            if (!boost::algorithm::equals(viewSurfaces, virtSurfaces2))
                continue;
            renderer.tilesetMapping = getTilesetMapping(MapConfig::convertPath(
                                                it.mapping, mapConfig->name));
            if (!testAndThrow(renderer.tilesetMapping->state,
                              "Tileset mapping failure."))
                return false;
            mapConfig->generateSurfaceStack(&it);
            renderer.tilesetMapping->update(virtSurfaces);
            break;
        }
    }

    if (mapConfig->surfaceStack.empty())
        mapConfig->generateSurfaceStack();

    renderer.traverseRoot = std::make_shared<TraverseNode>(nullptr, NodeInfo(
                    mapConfig->referenceFrame, TileId(), false, *mapConfig));
    renderer.traverseRoot->priority = std::numeric_limits<double>::infinity();
    renderer.credits.merge(mapConfig.get());
    initializeNavigation();
    mapConfig->initializeCelestialBody();

    LOG(info3) << "Map config ready";
    initialized = true;
    if (callbacks.mapconfigReady)
        callbacks.mapconfigReady();
    return initialized;
}

void MapImpl::renderTickPrepare()
{
    if (!prerequisitesCheck())
        return;

    assert(!resources.auth || *resources.auth);
    assert(mapConfig && *mapConfig);
    assert(convertor);
    assert(renderer.traverseRoot);

    updateNavigation();
    updateSearch();
    updateSris();
    traverseClearing(renderer.traverseRoot.get());
}

void MapImpl::renderCamera()
{
    vec3 center, dir, up;
    positionToCamera(center, dir, up);

    vtslibs::registry::Position &pos = mapConfig->position;

    // camera view matrix
    double dist = pos.type == vtslibs::registry::Position::Type::objective
            ? positionObjectiveDistance() : 1e-5;
    vec3 cameraPosPhys = center - dir * dist;
    if (callbacks.cameraOverrideEye)
        callbacks.cameraOverrideEye((double*)&cameraPosPhys);
    if (callbacks.cameraOverrideTarget)
        callbacks.cameraOverrideTarget((double*)&center);
    if (callbacks.cameraOverrideUp)
        callbacks.cameraOverrideUp((double*)&up);
    mat4 view = lookAt(cameraPosPhys, center, up);
    if (callbacks.cameraOverrideView)
    {
        callbacks.cameraOverrideView((double*)&view);
        // update center, dir and up
        mat4 vi = view.inverse();
        cameraPosPhys = vec4to3(vi * vec4(0, 0, -1, 1), true);
        dir = vec4to3(vi * vec4(0, 0, -1, 0), false);
        up = vec4to3(vi * vec4(0, 1, 0, 0), false);
        center = cameraPosPhys + dir * dist;
    }

    // camera projection matrix
    double near = std::max(2.0, dist * 0.1);
    double terrainAboveOrigin = 0;
    double cameraAboveOrigin = 0;
    switch (mapConfig->navigationSrsType())
    {
    case vtslibs::registry::Srs::Type::projected:
    {
        const vtslibs::registry::Srs &srs = mapConfig->srs.get(
                    mapConfig->referenceFrame.model.navigationSrs);
        if (srs.periodicity)
            terrainAboveOrigin = srs.periodicity->period / (2 * 3.14159);
        cameraAboveOrigin = terrainAboveOrigin + dist * 2;
    } break;
    case vtslibs::registry::Srs::Type::geographic:
    {
        terrainAboveOrigin = length(convertor->navToPhys(vec2to3(vec3to2(
                                    vecFromUblas<vec3>(pos.position)), 0)));
        cameraAboveOrigin = length(cameraPosPhys);
    } break;
    case vtslibs::registry::Srs::Type::cartesian:
        LOGTHROW(fatal, std::invalid_argument) << "Invalid navigation srs type";
    }
    double cameraToHorizon = cameraAboveOrigin > terrainAboveOrigin
            ? std::sqrt(cameraAboveOrigin * cameraAboveOrigin
                - terrainAboveOrigin * terrainAboveOrigin)
            : 0;
    double mountains = 5000 + terrainAboveOrigin;
    double mountainsBehindHorizon = std::sqrt(mountains * mountains
                                    - terrainAboveOrigin * terrainAboveOrigin);
    double far = cameraToHorizon + mountainsBehindHorizon;
    far = std::max(far, near * 10);
    double fov = pos.verticalFov;
    double aspect = (double)renderer.windowWidth/(double)renderer.windowHeight;
    if (callbacks.cameraOverrideFovAspectNearFar)
        callbacks.cameraOverrideFovAspectNearFar(fov, aspect, near, far);
    assert(fov > 1e-3 && fov < 180 - 1e-3);
    assert(aspect > 0);
    assert(near > 0);
    assert(far > near);
    mat4 proj = perspectiveMatrix(fov, aspect, near, far);
    if (callbacks.cameraOverrideProj)
        callbacks.cameraOverrideProj((double*)&proj);

    // few other variables
    renderer.viewProjRender = proj * view;
    if (!options.debugDetachedCamera)
    {
        renderer.viewProj = renderer.viewProjRender;
        renderer.perpendicularUnitVector
                = normalize(cross(cross(up, dir), dir));
        renderer.forwardUnitVector = dir;
        frustumPlanes(renderer.viewProj, renderer.frustumPlanes);
        renderer.cameraPosPhys = cameraPosPhys;
        renderer.focusPosPhys = center;
    }
    else
    {
        // render original camera
        RenderTask task;
        task.mesh = getMeshRenderable("internal://data/meshes/line.obj");
        task.mesh->priority = std::numeric_limits<float>::infinity();
        task.color = vec4f(0, 1, 0, 1);
        if (task.ready())
        {
            std::vector<vec3> corners;
            corners.reserve(8);
            mat4 m = renderer.viewProj.inverse();
            for (int x = 0; x < 2; x++)
                for (int y = 0; y < 2; y++)
                    for (int z = 0; z < 2; z++)
                        corners.push_back(vec4to3(m
                            * vec4(x * 2 - 1, y * 2 - 1, z * 2 - 1, 1), true));
            static const uint32 cora[] = {
                0, 0, 1, 2, 4, 4, 5, 6, 0, 1, 2, 3
            };
            static const uint32 corb[] = {
                1, 2, 3, 3, 5, 6, 7, 7, 4, 5, 6, 7
            };
            for (uint32 i = 0; i < 12; i++)
            {
                vec3 a = corners[cora[i]];
                vec3 b = corners[corb[i]];
                task.model = lookAt(a, b);
                draws.Infographic.emplace_back(task, this);
            }
        }
    }

    // render object position
    if (options.debugRenderObjectPosition)
    {
        vec3 phys = convertor->navToPhys(vecFromUblas<vec3>(pos.position));
        RenderTask r;
        r.mesh = getMeshRenderable("internal://data/meshes/cube.obj");
        r.mesh->priority = std::numeric_limits<float>::infinity();
        r.textureColor = getTexture("internal://data/textures/helper.jpg");
        r.textureColor->priority = std::numeric_limits<float>::infinity();
        r.model = translationMatrix(phys)
                * scaleMatrix(pos.verticalExtent * 0.015);
        if (r.ready())
            draws.Infographic.emplace_back(r, this);
    }

    // render target position
    if (options.debugRenderTargetPosition)
    {
        vec3 phys = convertor->navToPhys(navigation.targetPoint);
        RenderTask r;
        r.mesh = getMeshRenderable("internal://data/meshes/cube.obj");
        r.mesh->priority = std::numeric_limits<float>::infinity();
        r.textureColor = getTexture("internal://data/textures/helper.jpg");
        r.textureColor->priority = std::numeric_limits<float>::infinity();
        r.model = translationMatrix(phys)
                * scaleMatrix(navigation.targetViewExtent * 0.015);
        if (r.ready())
            draws.Infographic.emplace_back(r, this);
    }

    // update draws camera
    {
        MapDraws::Camera &c = draws.camera;
        matToRaw(view, c.view);
        matToRaw(proj, c.proj);
        vecToRaw(center, c.target);
        vecToRaw(cameraPosPhys, c.eye);
        c.near = near;
        c.far = far;
    }
}

void MapImpl::renderTickRender()
{
    draws.clear();

    if (!initialized || mapConfig->surfaceStack.empty()
            || renderer.windowWidth == 0 || renderer.windowHeight == 0)
        return;

    resources.gridTexture = getTexture("internal://data/textures/helper.jpg");
    resources.gridTexture->priority = std::numeric_limits<float>::infinity();

    renderCamera();
    traverseRender();
    travPreloadNodes();
    renderer.credits.tick(credits);
    for (const RenderTask &r : navigation.renders)
        draws.Infographic.emplace_back(r, this);
}

void MapImpl::applyCameraRotationNormalization(vec3 &rot)
{
    if (!options.enableCameraNormalization
            || options.navigationType == NavigationType::FlyOver)
        return;

    // find the interpolation factor
    double extCur = mapConfig->position.verticalExtent;
    double extLow = options.viewExtentThresholdScaleLow * body.majorRadius;
    double extHig = options.viewExtentThresholdScaleHigh * body.majorRadius;
    extCur = std::log2(extCur);
    extLow = std::log2(extLow);
    extHig = std::log2(extHig);
    double f = (extCur - extLow) / (extHig - extLow);
    f = clamp(f, 0, 1);
    f = smootherstep(f);

    // tilt limit
    rot(1) = interpolate(rot(1), options.tiltLimitAngleLow, f);

    // yaw limit
    double &yaw = rot(0);
    if (options.navigationMode == NavigationMode::Azimuthal)
        yaw = 0;
    else if (options.navigationMode == NavigationMode::Seamless)
    {
        if (yaw > 180)
            yaw = 360 - interpolate(360 - yaw, 0, f);
        else
            yaw = interpolate(yaw, 0, f);
    }
}

double MapImpl::travDistance(TraverseNode *trav, const vec3 pointPhys)
{
    if (!vtslibs::vts::empty(trav->meta->geomExtents)
            && !trav->nodeInfo.srs().empty())
    {
        // todo periodicity
        vec2 fl = vecFromUblas<vec2>(trav->nodeInfo.extents().ll);
        vec2 fu = vecFromUblas<vec2>(trav->nodeInfo.extents().ur);
        vec3 el = vec2to3(fl, trav->meta->geomExtents.z.min);
        vec3 eu = vec2to3(fu, trav->meta->geomExtents.z.max);
        vec3 p = convertor->convert(pointPhys,
            Srs::Physical, trav->nodeInfo.node());
        return aabbPointDist(p, el, eu);
    }
    return aabbPointDist(pointPhys, trav->meta->aabbPhys[0],
            trav->meta->aabbPhys[1]);
}

float MapImpl::computeResourcePriority(TraverseNode *trav)
{
    if (options.traverseMode == TraverseMode::Hierarchical)
        return 1.f / trav->nodeInfo.distanceFromRoot();
    if ((trav->hash + renderer.tickIndex) % 4 == 0) // skip expensive function
        return (float)(1e6 / (travDistance(trav, renderer.focusPosPhys) + 1));
    return trav->priority;
}

double MapImpl::getMapRenderProgress()
{
    uint32 active = statistics.currentResourcePreparing;
    if (active == 0)
    {
        resources.progressEstimationMaxResources = 0;
        return 0;
    }

    resources.progressEstimationMaxResources
            = std::max(resources.progressEstimationMaxResources, active);
    return double(resources.progressEstimationMaxResources - active)
            / resources.progressEstimationMaxResources;
}

std::shared_ptr<Resource> MapImpl::preloadInternalTexture(TraverseNode *trav,
                                                       uint32 subMeshIndex)
{
    UrlTemplate::Vars vars(trav->nodeInfo.nodeId(),
            vtslibs::vts::local(trav->nodeInfo), subMeshIndex);
    std::shared_ptr<Resource> res = getTexture(
                trav->meta->surface->surface->urlIntTex(vars));
    res->updatePriority(trav->priority);
    return res;
}

} // namespace vts
