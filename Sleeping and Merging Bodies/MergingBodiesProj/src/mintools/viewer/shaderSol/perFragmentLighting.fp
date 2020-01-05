
uniform sampler2D shadowMap; 
uniform float sigma;

varying vec3 N;  // surface normal in camera 
varying vec3 v;  // surface fragment location in camera 
varying vec4 vL; // surface fragment location in light view NDC
 
void main(void) {

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
    float distanceFromLight = texture2D( shadowMap, vLnr.xy ).r + sigma; // avoid self shadowing
    float shadow = 1.0;
    if ( vL.w > 0.0 ) {
       shadow = distanceFromLight < vLnr.z ? 0.1 : 1.0 ;
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