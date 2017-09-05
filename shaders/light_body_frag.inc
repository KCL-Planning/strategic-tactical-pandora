	vec4 normalVector = N;
	if (N != vec4(0, 0, 0, 0))
	{
		normalVector = normalize(N);
	}
	
	for (int i = 0; i < lights.length(); ++i)
	{
		if (!lights[i].enabled)
		{
			continue;
		}
		
		vec4 halfVector = normalize(H[i]);
		vec4 lightVector = normalize(L[i]);
		
		float dotValue = max(dot(normalVector, lightVector), 0.0);
		if (dotValue > 0.0)
		{
			float distance = length(lights[i].position - worldPos);
			float intensity = 1.0;
			// TODO: AMD cards (at least under Linux) fuck this up!
			//if (lights[i].type != 0)
				intensity = 1.0 / (lights[i].constant_attenuation + lights[i].linear_attenuation * distance + lights[i].quadratic_attenuation * distance * distance);
			vec4 ambient = material_ambient * lights[i].ambient;
			
			// Check if the fact is within the 'cone' of the light. This only
			// applies to spot lights.
			bool inLight = true;
			
			// TODO: AMD cards (at least under Linux) fuck this up!
			if (lights[i].type == 1)
			{
				vec3 nLightToVertex = vec3(normalize(worldPos - lights[i].position));
				float angleLightToFrag = dot(nLightToVertex, normalize(lights[i].direction));
				float radLightAngle = lights[i].light_angle * 3.141592 / 180.0;
			
				if (angleLightToFrag < cos(radLightAngle))
				{
					inLight = false;
				}
			}
			
			if (inLight)
			{
				float shadowf = textureProj(depth_texture[i], shadow_coord[i]);
				//if (lights[i].type != 1) shadowf = 1;

				vec4 diffuse = dotValue * lights[i].diffuse * material_diffuse;
				vec4 specular = pow(max(dot(normalVector, halfVector), 0.0), 10.0) * material_specular * lights[i].specular;
				outColor += intensity * shadowf * (ambient + diffuse + specular * 100);
			}
			
			outColor += intensity * ambient;
		}
	}
	
	outColor += material_emissive;
	