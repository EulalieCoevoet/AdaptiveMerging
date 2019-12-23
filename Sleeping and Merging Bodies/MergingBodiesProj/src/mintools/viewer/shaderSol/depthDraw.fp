
uniform float alpha;

uniform sampler2D depthTexture; 

void main(void) {
    vec4 shadowcolor = texture2D( depthTexture, gl_TexCoord[0].xy );
    gl_FragColor.rgba = vec4(shadowcolor.r,shadowcolor.g,shadowcolor.b, alpha);
}