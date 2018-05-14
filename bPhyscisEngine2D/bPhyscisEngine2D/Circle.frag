#version 330 core

// Interpolated values from the vertex shaders
in vec2 UV;

// Ouput data
out vec4 color;
in vec3 mVertPositionInModelSpace;

// Values that stay constant for the whole mesh.
uniform sampler2D myTextureSampler;

void main()
{

	// Output color = red 
	//color = mVertPositionInModelSpace.xyz;
	//color = vec3(1,0,0).xyz;
	color = texture( myTextureSampler, UV ).rgba;

}