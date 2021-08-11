uniform float uVertexScale;

attribute vec2 aPosition;
attribute vec3 aColor;
attribute vec2 aTexCoord0, aTexCoord1;

varying vec3 vColor;
varying vec2 vTexCoord0, vTexCoord1;

void main() {

  if(uVertexScale > 1) // height > width
  {
	  gl_Position = vec4(aPosition.x, aPosition.y / uVertexScale, 0, 1);
  }
  else
  {
	  gl_Position = vec4(aPosition.x * uVertexScale, aPosition.y, 0, 1);
  }
  vColor = aColor;
  vTexCoord0 = aTexCoord0;
  vTexCoord1 = aTexCoord1;
}
