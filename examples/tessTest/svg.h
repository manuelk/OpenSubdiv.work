
//
//   .oooooo.    ooooo
//  d8P'  `Y8b   `888'                  Yb        .oooo.o oooo    ooo  .oooooooo
// 888            888                    `Yb     d88(  "8  `88.  .8'  888' `88b
// 888            888           8888888    `Yb   `"Y88b.    `88..8'   888   888
// 888     ooooo  888                      .dP   o.  )88b    `888'    `88bod8P'
// `88.    .88'   888       o   8888888  .dP     8""888P'     `8'     `8oooooo.
//  `Y8bood8P'   o888ooooood8           dP                            d"     YD
//                                                                     "Y88888P'
//
//
//
// Usage example
//
// main_render_loop {
//
//     Svg * svg = Svg::Create("screenshot.svg");
//
//     svg->SetPointSize(2.5f);
//     svg->SetPointColor(0.9f, 0.3f, 0.2f);
//     svg->SetLineWidth(1.0f);
//     svg->SetLineColor(0.463f, 0.725f, 0.0f);
//
//     draw some stuff
//
//     svg->Write();
//
//     draw some more stuff
//
//     svg->SetLineColor(0.0f, 0.0f, 0.0f);
//     svg->Write();
//
//     delete svg;
//
// }
//

#include "../common/glUtils.h"

#include <cassert>
#include <cstdio>

#if _MSC_VER && !defined(snprintf)
    #define snprintf _snprintf
#endif


class Svg {

public:

    static Svg * Create(char const * filename);

    void Write();

    void SetPointSize(float r);

    void SetLineWidth(float w);

    void SetPointColor(float r, float g, float b, float a=1.0f);

    void SetLineColor(float r, float g, float b, float a=1.0f);

    ~Svg();

private:

    struct Vertex {
        GLfloat pos[3];

        void debug() const;
    };

    friend bool operator==(Vertex const & a, Vertex const & b) {
        return (a.pos[0]==b.pos[0]) && (a.pos[1]==b.pos[1]) && (a.pos[2]==b.pos[2]);
    }

    struct Settings {

        Settings();

        float pointSize,
              lineWidth;

        float pointColor[4],
              lineColor[4];
    } settings;

private:

    void writeHeader() const;

    void writeFooter() const;

    GLfloat const * writePoint(GLfloat const * ptr) const;

    GLfloat const *  writeLine(GLfloat const * ptr) const;

    GLfloat const *  writePolyLine(GLfloat const * ptr, int nverts) const;

    static char const * getColorString(GLfloat const * rgba);

    void parseFeedbackBuffer(GLsizei size) const;

private:

    Svg() : _file(0), _feedbackBuffer(0) { }

    FILE * _file;

    GLint _viewport[4];

    static const GLsizei _feedbackBufferSize = 1024 * 1024 * 8;
    GLfloat * _feedbackBuffer;
};


Svg::Settings::Settings() {
    pointSize = 5.0;
    lineWidth = 1.0;
    memset(pointColor, 0, 4 * sizeof(float));
    memset(lineColor, 0, 4 * sizeof(float));
}

void Svg::SetPointSize(float r) {
    settings.pointSize = r;
}

void Svg::SetLineWidth(float w) {
    settings.lineWidth = w;
}

void Svg::SetPointColor(float r, float g, float b, float a) {
    settings.pointColor[0] = r;
    settings.pointColor[1] = g;
    settings.pointColor[2] = b;
    settings.pointColor[3] = a;
 }

void Svg::SetLineColor(float r, float g, float b, float a) {
    settings.lineColor[0] = r;
    settings.lineColor[1] = g;
    settings.lineColor[2] = b;
    settings.lineColor[3] = a;
 }


Svg * Svg::Create(char const * filename) {

    Svg * svg = new Svg;

    glGetIntegerv(GL_VIEWPORT, svg->_viewport);
    if(!svg->_viewport[2] || !svg->_viewport[3]){
        printf("Svg: incorrect GL viewport\n");
        goto fail;
    }

    GLsizei size = _feedbackBufferSize;

    svg->_feedbackBuffer = new GLfloat[size];

    glFeedbackBuffer(size, GL_3D, svg->_feedbackBuffer);

    glRenderMode(GL_FEEDBACK);

    svg->_file = fopen(filename, "w");
    if (!svg->_file) {
        printf("cannot open '%s'\n", filename);
        goto fail;
    }
    printf("Writing : '%s'\n", filename); fflush(stdout);
    svg->writeHeader();

    return svg;

fail:
    delete svg;
    return 0;
}


Svg::~Svg() {

    if (_file) {
        writeFooter();
        fclose(_file);
    }

    delete [] _feedbackBuffer;

    glRenderMode(GL_RENDER);
}

void Svg::Write() {

    GLint size = glRenderMode(GL_FEEDBACK);

    if (size>0) {
        parseFeedbackBuffer(size);
    }
}

void Svg::parseFeedbackBuffer(GLsizei size) const {

    assert(_feedbackBuffer);

    GLfloat const * ptr = _feedbackBuffer;

    while (ptr < _feedbackBuffer + size) {

        GLint token = (GLint)*ptr++;

        switch (token) {

            case GL_POINT_TOKEN: {
                ptr = writePoint(ptr);
            } break;

            case GL_LINE_TOKEN:
            case GL_LINE_RESET_TOKEN: {
                ptr = writeLine(ptr);
            } break;

            case GL_POLYGON_TOKEN:
            case GL_BITMAP_TOKEN :
            case GL_DRAW_PIXEL_TOKEN :
            case GL_COPY_PIXEL_TOKEN :
            case GL_PASS_THROUGH_TOKEN:
            default:
                printf("unsupported token : %x\n", token);
                break;
        }
    }
}

void Svg::Vertex::debug() const {
    printf("pos=(%f %f %f)\n", pos[0], pos[1], pos[2]);
}

void Svg::writeHeader() const {

    assert(_file);

    int x      = _viewport[0],
        y      = _viewport[1],
        width  = _viewport[2],
        height = _viewport[3];

    fprintf(_file, "<svg width=\"%dpx\" height=\"%dpx\" viewBox=\"%d %d %d %d\">\n",
        width, height, x, y, width, height);
}

void Svg::writeFooter() const {
    fprintf(_file, "</svg>\n");
}

GLfloat const * Svg::writePoint(GLfloat const * ptr) const {

    Vertex const * v = (Vertex const *)ptr;

    float y = (float)_viewport[3],
          radius = settings.pointSize,
          opacity = settings.pointColor[3];

    char const * c = getColorString(settings.pointColor);

    fprintf(_file, "  <circle cx=\"%g\" cy=\"%g\" r=\"%g\" "
        "style=\"fill:%s;fill-opacity:%g;stroke:none\"/>\n",
            v->pos[0], y-v->pos[1], radius, c, opacity);

   return ptr + (sizeof(Vertex)/sizeof(GLfloat));
}

GLfloat const *  Svg::writeLine(GLfloat const * ptr) const {

    assert(ptr);

    Vertex const * v = (Vertex const *)ptr;

    float y = (float)_viewport[3],
          width = settings.lineWidth,
          opacity = settings.lineColor[3];

    char const * c = getColorString(settings.lineColor);

//    fprintf(_file, "  <line x1=\"%g\" y1=\"%g\" x2=\"%g\" y2=\"%g\" "
//        "style=\"stroke:%s;stroke-width:%g;stroke-opacity:%g;fill:none\"/>\n",
//            v[0].pos[0], y-v[0].pos[1], v[1].pos[0], y-v[1].pos[1], c, width, opacity);

    fprintf(_file, "  <path d=\"M %g %g L %g %f\" "
//        "stroke=%s stroke-width=%g stroke-opacity=%g fill=\"none\" />\n",
        "style=\"fill:none;stroke:%s;stroke-width:%g;stroke-opacity:%g\"/>\n",
            v[0].pos[0], y-v[0].pos[1], v[1].pos[0], y-v[1].pos[1], c, width, opacity);

    return ptr + 2 * (sizeof(Vertex)/sizeof(GLfloat));
}

GLfloat const *  Svg::writePolyLine(GLfloat const * ptr, int nverts) const {

    Vertex const * v = (Vertex const *)ptr;

    float y = (float)_viewport[3],
          width = settings.lineWidth,
          opacity = settings.lineColor[3];

    char const * c = getColorString(settings.lineColor);

    fprintf(_file, "  <polyline points=\"");
    for (int i=0; i<nverts; ++i, ptr+=3) {
         Vertex const * v = (Vertex const *)ptr;
         fprintf(_file, "%g,%g ", v->pos[0], v->pos[1]);
    }

    fprintf(_file, "\" style=\"fill:none;stroke:%s;stroke-width:%g;stroke-opacity:%g\"/>\n",
        c, width, opacity);

    return ptr;
}

char const * Svg::getColorString(GLfloat const * rgba) {

    static char buf[32];

    int r = (int)(255. * rgba[0]); r = (r < 0) ? 0 : (r > 255) ? 255 : r;
    int g = (int)(255. * rgba[1]); g = (g < 0) ? 0 : (g > 255) ? 255 : g;
    int b = (int)(255. * rgba[2]); b = (b < 0) ? 0 : (b > 255) ? 255 : b;

    snprintf(buf, 32, "#%2.2x%2.2x%2.2x", r, g, b);

    return buf;
}
