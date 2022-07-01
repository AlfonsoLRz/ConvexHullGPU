#version 450

layout (points) in;
layout (line_strip, max_vertices = 2) out;

in VS_OUT {
    vec4 normal;
    vec3 color;
} gs_in[];

uniform float normalLength;

out vec3 color;

void main() {
    gl_Position = gl_in[0].gl_Position;
    color = gs_in[0].color;

    EmitVertex();

    gl_Position = gl_in[0].gl_Position + gs_in[0].normal * normalLength;
    color = gs_in[0].color;

    EmitVertex();

    EndPrimitive();
}