// Ponte C -> Syphon: publica a imagem para outros programas (OBS, Resolume, MadMapper...).
// O programa entrega os pixels (RGBA, de baixo para cima, como o OpenGL le); aqui eles viram
// uma textura comum que o Syphon publica. Caminho simples e testado com um cliente Syphon
// (imagem completa, 1920x1080, com alfa); a copia pela CPU custa pouco a 30 quadros/s.
#define GL_SILENCE_DEPRECATION
#import <Cocoa/Cocoa.h>
#import <OpenGL/OpenGL.h>
#import <OpenGL/gl.h>
#import <OpenGL/glext.h>
#import <Syphon/SyphonOpenGLServer.h>

static SyphonOpenGLServer *server = nil;
static GLuint tex = 0;
static int tex_w = 0, tex_h = 0;

int syphon_start(const char *name) {
    CGLContextObj ctx = CGLGetCurrentContext();
    if (!ctx) return 0;
    server = [[SyphonOpenGLServer alloc] initWithName:[NSString stringWithUTF8String:name] context:ctx options:nil];
    return server != nil;
}

int syphon_clients(void) {
    return server && server.hasClients ? 1 : 0;
}

void syphon_publish_pixels(const void *rgba, int w, int h) {
    if (!server) return;
    if (!tex || w != tex_w || h != tex_h) {
        if (!tex) glGenTextures(1, &tex);
        glBindTexture(GL_TEXTURE_2D, tex);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
        tex_w = w; tex_h = h;
    }
    glBindTexture(GL_TEXTURE_2D, tex);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, rgba);
    glBindTexture(GL_TEXTURE_2D, 0);
    glFlush();

    // o Syphon desenha com o mesmo contexto: guarda e restaura o estado para nao afetar a janela
    GLint fb = 0, vp[4];
    glGetIntegerv(GL_FRAMEBUFFER_BINDING_EXT, &fb);
    glGetIntegerv(GL_VIEWPORT, vp);
    glPushAttrib(GL_ALL_ATTRIB_BITS);
    glPushClientAttrib(GL_CLIENT_ALL_ATTRIB_BITS);
    glMatrixMode(GL_TEXTURE); glPushMatrix();
    glMatrixMode(GL_PROJECTION); glPushMatrix();
    glMatrixMode(GL_MODELVIEW); glPushMatrix();
    glDisable(GL_DEPTH_TEST); glDisable(GL_BLEND);
    glColor4f(1, 1, 1, 1);

    [server publishFrameTexture:tex textureTarget:GL_TEXTURE_2D imageRegion:NSMakeRect(0, 0, w, h)
              textureDimensions:NSMakeSize(w, h) flipped:NO];
    glFlush();

    glMatrixMode(GL_TEXTURE); glPopMatrix();
    glMatrixMode(GL_PROJECTION); glPopMatrix();
    glMatrixMode(GL_MODELVIEW); glPopMatrix();
    glPopClientAttrib();
    glPopAttrib();
    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fb);
    glViewport(vp[0], vp[1], vp[2], vp[3]);
    glBindTexture(GL_TEXTURE_2D, 0);
    glUseProgram(0);
}

void syphon_stop(void) {
    [server stop]; server = nil;
}
