export SDL_VIDEO_GL_DRIVER=/usr/lib/dri/v3d_dri.so 
export LIBGL_DRIVERS_PATH=/usr/lib/dri
export MESA_GLSL_VERSION_OVERRIDE=450
export MESA_GL_VERSION_OVERRIDE=4.5
cd /root
./@template@ &> /dev/null