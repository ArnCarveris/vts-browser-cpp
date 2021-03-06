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
    convertor.reset();
    body = MapCelestialBody();
    mapconfigAvailable = false;
    purgeViewCache();
}

void MapImpl::purgeViewCache()
{
    LOG(info2) << "Purge view cache";

    if (mapConfig)
        mapConfig->consolidateView();

    layers.clear();
    statistics.resetFrame();
    draws = MapDraws();
    credits = MapCredits();
    mapConfigView = "";
    mapconfigReady = false;
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
    for (auto &it : trav->opaque)
        touchDraws(it);
    for (auto &it : trav->transparent)
        touchDraws(it);
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

void MapImpl::renderNode(TraverseNode *trav, const vec4f &uvClip)
{
    assert(trav->meta);
    assert(!trav->rendersEmpty());
    assert(trav->rendersReady());
    assert(visibilityTest(trav));

    // statistics
    statistics.meshesRenderedTotal++;
    statistics.meshesRenderedPerLod[std::min<uint32>(
        trav->nodeInfo.nodeId().lod, MapStatistics::MaxLods - 1)]++;

    // meshes
    if (!options.debugRenderNoMeshes)
    {
        for (const RenderTask &r : trav->opaque)
            draws.opaque.emplace_back(r, uvClip.data(), this);
        for (const RenderTask &r : trav->transparent)
            draws.transparent.emplace_back(r, uvClip.data(), this);
    }

    // surrogate
    if (options.debugRenderSurrogates)
    {
        RenderTask task;
        task.mesh = getMeshRenderable("internal://data/meshes/sphere.obj");
        task.mesh->priority = std::numeric_limits<float>::infinity();
        task.model = translationMatrix(trav->meta->surrogatePhys)
                * scaleMatrix(trav->nodeInfo.extents().size() * 0.03);
        if (trav->surface)
            task.color = vec3to4f(trav->surface->color, task.color(3));
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
            task.mesh = getMeshRenderable("internal://data/meshes/aabb.obj");
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

    // credits
    for (auto &it : trav->meta->credits)
        renderer.credits.hit(Credits::Scope::Imagery, it,
                             trav->nodeInfo.distanceFromRoot());

    trav->lastRenderTime = renderer.tickIndex;
}

namespace
{

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

void MapImpl::renderNodePartialRecursive(TraverseNode *trav, vec4f uvClip)
{
    if (!trav->parent || !trav->parent->surface)
        return;

    auto id = trav->nodeInfo.nodeId();
    float *arr = uvClip.data();
    updateRangeToHalf(arr[0], arr[2], id.x % 2);
    updateRangeToHalf(arr[1], arr[3], 1 - (id.y % 2));

    if (!trav->parent->rendersEmpty() && trav->parent->rendersReady())
        renderNode(trav->parent, uvClip);
    else
        renderNodePartialRecursive(trav->parent, uvClip);
}

bool MapImpl::prerequisitesCheck()
{
    if (resources.auth)
        resources.auth->checkTime();

    if (mapconfigReady)
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

    if (!convertor)
    {
        convertor = CoordManip::create(
                    *mapConfig,
                    mapConfig->browserOptions.searchSrs,
                    createOptions.customSrs1,
                    createOptions.customSrs2);
    }

    if (!mapconfigAvailable)
    {
        LOG(info3) << "Map config is available.";
        mapconfigAvailable = true;
        if (callbacks.mapconfigAvailable)
        {
            callbacks.mapconfigAvailable();
            return false;
        }
    }

    if (layers.empty())
    {
        // main surface stack
        layers.push_back(std::make_shared<MapLayer>(this));

        // free layers
        for (const auto &it : mapConfig->view.freeLayers)
            layers.push_back(std::make_shared<MapLayer>(this, it.first,
                                                        it.second));
    }

    // check all layers
    {
        bool ok = true;
        for (auto &it : layers)
        {
            if (!it->prerequisitesCheck())
                ok = false;
        }
        if (!ok)
            return false;
    }

    renderer.credits.merge(mapConfig.get());
    initializeNavigation();
    mapConfig->initializeCelestialBody();

    LOG(info2) << "Map config is ready.";
    mapconfigReady = true;
    if (callbacks.mapconfigReady)
        callbacks.mapconfigReady(); // this may change initialized state
    return mapconfigReady;
}

void MapImpl::renderTickPrepare()
{
    if (!prerequisitesCheck())
        return;

    assert(!resources.auth || *resources.auth);
    assert(mapConfig && *mapConfig);
    assert(convertor);
    assert(!layers.empty());
    assert(layers[0]->traverseRoot);

    updateNavigation();
    updateSearch();
    updateSris();
    for (auto &it : layers)
        traverseClearing(it->traverseRoot.get());
}

void MapImpl::renderTickRender()
{
    draws.clear();

    if (!mapconfigReady || renderer.windowWidth == 0 || renderer.windowHeight == 0)
        return;

    updateCamera();
    for (auto &it : layers)
        traverseRender(it->traverseRoot.get());
    renderer.credits.tick(credits);
    for (const RenderTask &r : navigation.renders)
        draws.Infographic.emplace_back(r, this);

    draws.sortOpaqueFrontToBack();
}

namespace
{

void computeNearFar(double &near, double &far, double altitude,
                    const MapCelestialBody &body, bool projected,
                    vec3 cameraPos, vec3 cameraForward)
{
    (void)cameraForward;
    double major = body.majorRadius;
    double flat = major / body.minorRadius;
    cameraPos[2] *= flat;
    double ground = major + (altitude == altitude ? altitude : 0.0);
    double l = projected ? cameraPos[2] + major : length(cameraPos);
    double a = std::max(1.0, l - ground);
    //LOG(info4) << "altitude: " << altitude << ", ground: " << ground
    //           << ", camera: " << l << ", above: " << a;

    if (a > 2 * major)
    {
        near = a - major;
        far = l;
    }
    else
    {
        double f = std::pow(a / (2 * major), 1.1);
        near = interpolate(10.0, major, f);
        near = std::max(10.0, near);
        far = std::sqrt(std::max(0.0, l * l - major * major)) + 0.1 * major;
    }
}

} // namespace

void MapImpl::updateCamera()
{
    bool projected = mapConfig->navigationSrsType()
            == vtslibs::registry::Srs::Type::projected;

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
        computeNearFar(near, far, altitude, body, projected,
                       cameraPos, cameraForward);
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
    renderer.viewRender = view;
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
        vecToRaw(cameraPos, c.eye);
        c.near = near;
        c.far = far;
        c.aspect = aspect;
        c.fov = fov;
        c.mapProjected = projected;
    }
}

} // namespace vts
