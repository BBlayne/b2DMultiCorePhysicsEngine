#version 330 core

// Ouput data
out vec3 color;
in vec3 mVertPositionInModelSpace;

// Interpolated values from the vertex shaders
in vec2 UV;

void main()
{

	// Output color = red 
	//color = mVertPositionInModelSpace.xyz;
	color = vec3(1,0,0).xyz;

}