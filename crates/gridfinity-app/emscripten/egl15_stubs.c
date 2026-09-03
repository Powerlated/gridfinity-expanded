/* The two EGL 1.5 entry points wgpu-hal names but never reaches on Emscripten.
 *
 * `wgpu_hal::gles::egl` compiles its EGL 1.5 arms unconditionally and, under
 * Emscripten, hands them an instance it has asserted is 1.5. It then guards
 * every one of them: the platform-display arms need a client extension
 * Emscripten does not advertise, and the platform-surface arm needs a window
 * kind other than `Unknown`, which is the only kind Emscripten produces. So the
 * calls are linked and unreachable, while Emscripten's own EGL stops at 1.4 and
 * defines neither symbol. These stubs make the link honest and abort loudly
 * rather than returning a plausible handle, so a wgpu that does start reaching
 * this path says so instead of failing somewhere further along.
 */
#include <stdio.h>
#include <stdlib.h>

static void *unreachable_egl15(const char *name) {
  fprintf(stderr, "%s: Emscripten's EGL is 1.4 and wgpu must not reach this path\n", name);
  abort();
}

void *eglGetPlatformDisplay(unsigned int platform, void *native_display,
                            const void *attrib_list) {
  (void)platform;
  (void)native_display;
  (void)attrib_list;
  return unreachable_egl15("eglGetPlatformDisplay");
}

void *eglCreatePlatformWindowSurface(void *display, void *config, void *native_window,
                                     const void *attrib_list) {
  (void)display;
  (void)config;
  (void)native_window;
  (void)attrib_list;
  return unreachable_egl15("eglCreatePlatformWindowSurface");
}
