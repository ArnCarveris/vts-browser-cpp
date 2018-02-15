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

void updateUvsForGrids(float *uvm, float *uvclip)
{
    static const float scale = 16;
    static const mat3f scm = mat4to3(scaleMatrix(scale)).cast<float>();
    {
        mat3f m = rawToMat3(uvm);
        m = m * scm;
        matToRaw(m, uvm);
    }
    for (int i = 0; i < 4; i++)
        uvclip[i] *= scale;
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

    travel.clear();

    renderer.tilesetMapping.reset();
    statistics.resetFrame();
    draws = MapDraws();
    credits = MapCredits();
    mapConfigView = "";
    initialized = false;
}

TileId MapImpl::roundIdByTileBinaryOrder(TileId nodeId)
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

    //renderer.traverseRoot = std::make_shared<TraverseNode>(nullptr, NodeInfo(
    //                mapConfig->referenceFrame, TileId(), false, *mapConfig));
    //renderer.traverseRoot->priority = std::numeric_limits<double>::infinity();

    initializeTravelRoot();

    renderer.credits.merge(mapConfig.get());
    initializeNavigation();
    mapConfig->initializeCelestialBody();

    LOG(info3) << "Map config ready";
    initialized = true;
    if (callbacks.mapconfigReady)
        callbacks.mapconfigReady();
    return initialized;
}

void MapImpl::renderTickPrepare(double timeStepSeconds)
{
    if (!prerequisitesCheck())
        return;

    assert(!resources.auth || *resources.auth);
    assert(mapConfig && *mapConfig);
    assert(convertor);
    //assert(renderer.traverseRoot);
    assert(travel.root);

    updateNavigation(timeStepSeconds);
    updateSearch();
    updateSris();
    //traverseClearing(renderer.traverseRoot.get());

    travelTickPrepare();
}

namespace
{

void computeNearFar(double &near, double &far, double altitude,
                    const MapCelestialBody &body,
                    vec3 cameraPos, vec3 cameraForward)
{
    (void)cameraForward;
    double major = body.majorRadius;
    double flat = major / body.minorRadius;
    cameraPos[2] *= flat;
    //cameraForward[2] *= flat;
    //cameraForward = normalize(cameraForward);
    double ground = major + (altitude == altitude ? altitude : 0.0);
    double l = length(cameraPos);
    double a = std::max(1.0, l - ground);
    //LOG(info4) << "altitude: " << altitude << ", ground: " << ground
    //           << ", camera: " << l << ", above: " << a;
    near = a > 2 * major ? a - major : interpolate(2, major, a / (2 * major));
    near = std::max(2.0, near);
    //double l2 = std::max(l, ground) + 5000;
    //double horizon = std::sqrt(l2*l2 - ground*ground);
    //far = near + horizon;
    far = l;
}

} // namespace

void MapImpl::renderCamera()
{
    vec3 objCenter, cameraForward, cameraUp;
    positionToCamera(objCenter, cameraForward, cameraUp);

    vtslibs::registry::Position &pos = mapConfig->position;

    // camera view matrix
    double objDist = pos.type == vtslibs::registry::Position::Type::objective
            ? positionObjectiveDistance() : 1e-5;
    vec3 cameraPos = objCenter - cameraForward * objDist;
    if (callbacks.cameraOverrideEye)
        callbacks.cameraOverrideEye(cameraPos.data());
    if (callbacks.cameraOverrideTarget)
        callbacks.cameraOverrideTarget(objCenter.data());
    objDist = length(vec3(objCenter - cameraPos));
    if (callbacks.cameraOverrideUp)
        callbacks.cameraOverrideUp(cameraUp.data());
    assert(length(cameraUp) > 1e-7);
    mat4 view = lookAt(cameraPos, objCenter, cameraUp);
    if (callbacks.cameraOverrideView)
    {
        callbacks.cameraOverrideView(view.data());
        // update objCenter, cameraForward and cameraUp
        mat4 vi = view.inverse();
        cameraPos = vec4to3(vi * vec4(0, 0, -1, 1), true);
        cameraForward = vec4to3(vi * vec4(0, 0, -1, 0), false);
        cameraUp = vec4to3(vi * vec4(0, 1, 0, 0), false);
        objCenter = cameraPos + cameraForward * objDist;
    }

    // camera projection matrix
    double near = 0;
    double far = 0;
    {
        double altitude;
        vec3 navPos = convertor->physToNav(cameraPos);
        if (!getPositionAltitude(altitude, navPos, 10))
            altitude = std::numeric_limits<double>::quiet_NaN();
        computeNearFar(near, far, altitude, body, cameraPos, cameraForward);
    }
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
        callbacks.cameraOverrideProj(proj.data());

    // few other variables
    renderer.viewProjRender = proj * view;
    if (!options.debugDetachedCamera)
    {
        renderer.viewProj = renderer.viewProjRender;
        renderer.perpendicularUnitVector
            = normalize(cross(cross(cameraUp, cameraForward), cameraForward));
        renderer.forwardUnitVector = cameraForward;
        frustumPlanes(renderer.viewProj, renderer.frustumPlanes);
        renderer.cameraPosPhys = cameraPos;
        renderer.focusPosPhys = objCenter;
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
        //vecToRaw(objCenter, c.target);
        vecToRaw(cameraPos, c.eye);
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

    resources.gridTexture = getTexture("internal://data/textures/grid.png");
    resources.gridTexture->priority = std::numeric_limits<float>::infinity();
    resources.gridTexture->flags = GpuTextureSpec::Flags(
        GpuTextureSpec::Mipmaps | GpuTextureSpec::Repeat);

    renderCamera();
    //traverseRender();
    //traversePreloadNodes();

    travelTickRender();

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

} // namespace vts
