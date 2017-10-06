#version 150

layout(location = 0) in vec4 vPos;

uniform mat4 WVP;

void main()
{
  gl_Position = WVP * vPos;
}