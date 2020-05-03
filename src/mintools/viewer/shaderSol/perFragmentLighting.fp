
uniform sampler2D shadowMap; 
uniform float sigma;

varying vec3 N;  // surface normal in camera 
varying vec3 v;  // surface fragment location in camera 
varying vec4 vL; // surface fragment location in light view NDC
//varying float clip_distance;  // add a second clipping plane if desired!

 
 // some help with shadow map antialiasing from :
 // http://www.opengl-tutorial.org/intermediate-tutorials/tutorial-16-shadow-mapping/
 vec2 poissonDisk[16] = vec2[]( 
   vec2( -0.94201624, -0.39906216 ), 
   vec2( 0.94558609, -0.76890725 ), 
   vec2( -0.094184101, -0.92938870 ), 
   vec2( 0.34495938, 0.29387760 ), 
   vec2( -0.91588581, 0.45771432 ), 
   vec2( -0.81544232, -0.87912464 ), 
   vec2( -0.38277543, 0.27676845 ), 
   vec2( 0.97484398, 0.75648379 ), 
   vec2( 0.44323325, -0.97511554 ), 
   vec2( 0.53742981, -0.47373420 ), 
   vec2( -0.26496911, -0.41893023 ), 
   vec2( 0.79197514, 0.19090188 ), 
   vec2( -0.24188840, 0.99706507 ), 
   vec2( -0.81409955, 0.91437590 ), 
   vec2( 0.19984126, 0.78641367 ), 
   vec2( 0.14383161, -0.14100790 ) 
);

// Returns a random number based on a vec3 and an int.
float random(vec3 seed, int i){
	vec4 seed4 = vec4(seed,i);
	float dot_product = dot(seed4, vec4(12.9898,78.233,45.164,94.673));
	return fract(sin(dot_product) * 43758.5453);
}

void main(void) {

	//if ( clip_distance < 0.0 ) discard;
        
    vec3 L = normalize( gl_LightSource[0].position.xyz - v ); 
    vec3 E = normalize( -v ); // we are in Eye Coordinates, so EyePos is (0,0,0)  
    vec3 R = normalize( -reflect(L,N) );

    //calculate Ambient Term:  
    vec4 Iamb = gl_FrontLightProduct[0].ambient;    
    //calculate Diffuse Term:  
    vec4 Idiff = gl_FrontLightProduct[0].diffuse * max( dot(N,L), 0.0 );
    Idiff = clamp( Idiff, 0.0, 1.0 );     
    // calculate Specular Term:
    vec4 Ispec = gl_FrontLightProduct[0].specular * pow( max( dot(R,E), 0.0 ), gl_FrontMaterial.shininess );
    Ispec = clamp( Ispec, 0.0, 1.0 ); 

    vec4 vLn = vL / vL.w ; // vL normalized by w
    vec3 vLnr = vLn.xyz * 0.5  + vec3( 0.5, 0.5, 0.5 );  // and then remapped into values from 0 to 1
    float shadow = 1.0;
    if ( vL.w > 0.0 ) {
 	   	for (int i=0;i<16;i++){
	 	   	int index = i;//int(16.0*random(gl_FragCoord.xyy, i))%16;
  			float distanceFromLight = texture2D( shadowMap, vLnr.xy + poissonDisk[index]/700.0 ).r + sigma; // avoid self shadowing
        	shadow -= distanceFromLight < vLnr.z ? (1.0/16.0) : 0 ;
    	}      
    }
    
	float d = length( gl_LightSource[0].position.xyz - v );
	float attenuation = 1.0 / ( gl_LightSource[0].constantAttenuation +
								gl_LightSource[0].linearAttenuation * d +
								gl_LightSource[0].quadraticAttenuation * d * d );

    // write Total Color:  emissive (sceneColor), ambient, diffuse and specular
    gl_FragColor = 
    	//gl_FrontLightModelProduct.sceneColor + 
    	Iamb + (Idiff + Ispec) * shadow * attenuation;        
}