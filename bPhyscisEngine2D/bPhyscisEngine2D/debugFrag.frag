#version 330 core

// Ouput data
out vec4 color;
in vec3 mVertPositionInModelSpace;

in vec3 randomColour;

void main()
{
	color = vec4(randomColour.xyz, 0.75);

}