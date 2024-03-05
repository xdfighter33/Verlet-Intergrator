#ifdef GL_ES
precision mediump float;
#endif

uniform vec2 u_resolution;
uniform float u_time;
uniform vec3 sphereColor;
void main() {
    // Normalize the fragment's position
    vec2 st = gl_FragCoord.st / u_resolution;

    // Calculate a color based on the object's velocity
    vec3 color = vec3(abs(sin(u_time)), abs(cos(u_time)), abs(sin(u_time + u_time * 0.5)));

    // Output the color
    gl_FragColor = vec4(color, 1.0);
}