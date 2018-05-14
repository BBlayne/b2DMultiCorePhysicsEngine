#version 330 core

// Interpolated values from the vertex shaders
in vec2 UV;

// Ouput data
out vec4 color;
in vec3 mVertPositionInModelSpace;

in vec3 randomColour;

// Values that stay constant for the whole mesh.
uniform sampler2D myTextureSampler;

void main()
{
	vec4 temp = texture( myTextureSampler, UV ).rgba;
	// Output color is the pixels in given texture.
	if (temp.a > 0 && temp.x > 0.85)
	{
		temp = vec4(randomColour, 1);
	}
	color = temp;

}