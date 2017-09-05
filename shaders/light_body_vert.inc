	if (a_Normal == vec3(0, 0, 0))
	{
		N = vec4(0, 0, 0, 0);
	}
	else
	{
		N = normalize(modelview_matrix * vec4(a_Normal, 0.0));
	}
	
	for (int i = 0; i < lights.length(); ++i)
	{
		if (!lights[i].enabled)
		{
			continue;
		}
		vec3 lightPos = (view_matrix * lights[i].position).xyz;
		shadow_coord[i] = lights[i].shadow_matrix * worldPos;

		// Diffuse lightning = (L dot N) X C X I.
		if (lights[i].type == 0)
			L[i] = normalize(vec4(-lights[i].direction, 0.0));
		else
			L[i] = normalize(vec4((lightPos - pos.xyz), 0.0));

		// Calculate the final color.
	
		// The half vector.
		if (lights[i].type == 0)
			H[i] = normalize(vec4((-lights[i].direction).xyz, 1.0) - pos);
		else
			H[i] = normalize(vec4((lightPos - pos.xyz).xyz, 1.0) - pos);
	}