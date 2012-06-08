// shadow.vert

varying vec4 shadow;

void main(void)
{
  vec3 position = vec3(gl_ModelViewMatrix * gl_Vertex);
  vec3 normal = normalize(gl_NormalMatrix * gl_Normal);
  vec3 light = normalize(gl_LightSource[0].position.xyz - position);
  float diffuse = dot(light, normal);
  
  gl_FrontColor = gl_LightSource[0].ambient * gl_Color;
  shadow = gl_FrontColor;
  if (diffuse > 0.0) {
    vec3 view = normalize(position);
    vec3 halfway = normalize(light - view);
    float specular = pow(max(dot(normal, halfway), 0.0), gl_FrontMaterial.shininess);
	vec4 temp = gl_LightSource[0].diffuse * gl_Color * diffuse;
	shadow += temp * 0.2;
    gl_FrontColor += temp + gl_FrontLightProduct[0].specular * specular;
  }
  
  gl_TexCoord[0] = gl_TextureMatrix[0] * gl_ModelViewMatrix * gl_Vertex;
  gl_Position = ftransform();
}