
#include "./snapshot.h"
#include <cassert>
#include <cstdio>
#include <string>

#if _MSC_VER
    #define snprintf _snprintf
#endif

struct Vertex {
    GLfloat pos[3];
    GLfloat color[4];

    void print() const {
        printf("pos=(%f %f %f) col=(%f %f %f %f)\n",
           pos[0], pos[1], pos[2], color[0], color[1], color[2], color[3]);
    }
};

enum PrimitiveType {
    PRIM_POINT,
    PRIM_LINE,
    PRIM_TRIANGLE,
    PRIM_QUAD,
    PRIM_POLY
};

struct Primitive {

    PrimitiveType type;
    
    int nverts;
    Vertex const * verts;
};


namespace svg {

    void getColorString(GLfloat const * rgba, char * buf, int bufsize) {
        int r = (int)(255. * rgba[0]);
        int g = (int)(255. * rgba[1]);
        int b = (int)(255. * rgba[2]);
        int rc = (r < 0) ? 0 : (r > 255) ? 255 : r;
        int gc = (g < 0) ? 0 : (g > 255) ? 255 : g;
        int bc = (b < 0) ? 0 : (b > 255) ? 255 : b;
        snprintf(buf, bufsize, "#%2.2x%2.2x%2.2x", rc, gc, bc);
    }

    void printLine(FILE * file, Vertex const * verts) {
verts[0].print(); verts[1].print();
        float width = 1.0;

        char col[32];
        getColorString(verts[0].color, col, 32);      

        fprintf(file, "  <line x1=\"%g\" y1=\"%g\" x2=\"%g\" y2=\"%g\" "
            "style=\"stroke:%s;stroke-width:%g;fill:none;stroke-opacity:1\"/>\n",
                verts[0].pos[0], verts[0].pos[1],
                    verts[1].pos[0], verts[1].pos[1],
                        col, width);
    }

    void printHeader(FILE * file, char const * title, GLint viewport[4]) {

        int x = viewport[0],
            y = viewport[1],
            width = viewport[2],
            height = viewport[3];
        fprintf(file, "<svg width=\"%dpx\" height=\"%dpx\" viewBox=\"%d %d %d %d\">\n",
            width, height, x, y, width, height);

        fprintf(file, "<title>%s</title>\n", title);
        fprintf(file, "<desc></desc>\n");
        fprintf(file, "<defs></defs>\n");
    }

    void printFooter(FILE * file) {
        fprintf(file, "</svg>\n");
    }
}


struct Snapshot::glSnapshot {

    ~glSnapshot () {
      delete [] feedbackBuffer;
    }

    int getVertex(Vertex * v, GLfloat * ptr);

    void parseFeedbackBuffer(FILE * file, GLint size);

    GLint viewport[4];
    static const GLsizei feedbackBufferSize = 1024 * 1024 * 8;

    GLfloat * feedbackBuffer;

    std::string filename;
};

int Snapshot::glSnapshot::getVertex(Vertex * v, GLfloat * ptr) {

   memcpy(v->pos, ptr, 3 * sizeof(GLfloat));
   //memcpy(v->color, ptr + 3, 4 * sizeof(GLfloat));
   memset(v->color, 0, 4 * sizeof(GLfloat));
   return 3;
}

void Snapshot::glSnapshot::parseFeedbackBuffer(FILE * file, GLint size) {

    Vertex verts[3];

    GLint used = size;

    GLfloat * current = feedbackBuffer;

    while (used>0) {

        GLint token = (GLint)*current;
        ++current;
        --used;

        switch (token) {

          case GL_POINT_TOKEN: {
              int n = getVertex(&verts[0], current);
              current += n; used -= n;
              // add point
          } break;

          case GL_LINE_TOKEN:
          case GL_LINE_RESET_TOKEN: {
              int n = getVertex(&verts[0], current);
              current += n; used -= n;
              n = getVertex(&verts[1], current);
              current += n; used -= n;

              svg::printLine(file, verts);
          } break;

          case GL_POLYGON_TOKEN: {
              int nverts = (GLint)*current;
              ++current;
              assert(nverts<=3);
              for (int i=0; i<nverts && used>0; ++i) {
                  int n = getVertex(&verts[i], current);
                  current += n;
                  used -= n;
              }
              // add triangle
          } break;

          case GL_BITMAP_TOKEN : {
          } break;

          case GL_DRAW_PIXEL_TOKEN : {
          } break;

          case GL_COPY_PIXEL_TOKEN : {
          } break;

          case GL_PASS_THROUGH_TOKEN: {
              GLint pt_token = (GLint)*current;
              ++current;
              --used;
          } break;

          default:
              printf("unsupported token : %x\n", (GLint)*current);
              break;
        }
    }
}

// -----------------------------------------------------------------------------

Snapshot::Snapshot(char const * filename) {

    glSnapshot * snap = new glSnapshot;

    snap->filename = filename;

    glGetIntegerv(GL_VIEWPORT, snap->viewport);
    if(!snap->viewport[2] || !snap->viewport[3]){
        printf("snapshot: incorrect viewport\n");
        exit(0);
    }

    GLsizei size = glSnapshot::feedbackBufferSize;
    snap->feedbackBuffer = new GLfloat[size];
    glFeedbackBuffer(size, GL_3D, snap->feedbackBuffer);
    glRenderMode(GL_FEEDBACK);

    _glSnapshot = snap;
}

Snapshot::~Snapshot() {

    glSnapshot * snap = _glSnapshot;

    GLint size = glRenderMode(GL_RENDER);

    if (size > 0) {

        FILE * file = fopen(snap->filename.c_str(), "w");
        if (!file) {
            printf("cannot open '%s'\n", snap->filename.c_str());
            exit(0);
        }

        svg::printHeader(file, snap->filename.c_str(), snap->viewport);

        snap->parseFeedbackBuffer(file, size);

        svg::printFooter(file);

        printf("Writing '%s' %d\n", snap->filename.c_str(), size); fflush(stdout);
    }

    delete _glSnapshot;
}







