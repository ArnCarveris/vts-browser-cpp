
uniform sampler2D texColor;
uniform sampler2D texMask;

uniform vec4 uniColor;
uniform vec4 uniUvClip;
uniform ivec4 uniFlags; // mask, monochromatic, flat shading, uv source

in vec2 varUvTex;
in vec2 varUvClip;
in vec3 varViewPosition;

layout(location = 0) out vec4 outColor;

void main()
{
    vec4 color = texture(texColor, varUvTex);

    // uv clipping
    if (varUvClip.x < uniUvClip.x
            || varUvClip.y < uniUvClip.y
            || varUvClip.x > uniUvClip.z
            || varUvClip.y > uniUvClip.w)
        discard;

    // mask
    if (uniFlags.x > 0)
    {
        if (texture(texMask, varUvTex).r < 0.5)
            discard;
    }

    // base color
    if (uniFlags.z > 0)
    {
        // flat shading
        vec3 n = normalize(cross(dFdx(varViewPosition),
                                 dFdy(varViewPosition)));
        outColor = vec4(vec3(max(n.z * 0.8, 0.0) + 0.125), 1.0);
    }
    else
    {
        // texture color
        if (uniFlags.y > 0)
            color = color.rrra; // monochromatic texture
        outColor = color;
    }
    outColor *= uniColor;

    // atmosphere
    float atmosphere = atmDensity(varViewPosition);
    outColor = atmColor(atmosphere, outColor);
}

