#include "fluidrenderer.h"


FluidRenderer::FluidRenderer()
{
}

FluidRenderer::FluidRenderer(FluidSimulation *sim) : _fluidsim(sim)
{
}


FluidRenderer::~FluidRenderer()
{
}

void FluidRenderer::update(float dt) {

}

void FluidRenderer::_setTransforms() {
    glm::mat4 trans_mat = glm::transpose(glm::mat4(glm::vec4(1.0, 0.0, 0.0, _tx),
                                                   glm::vec4(0.0, 1.0, 0.0, _ty),
                                                   glm::vec4(0.0, 0.0, 1.0, _tz),
                                                   glm::vec4(0.0, 0.0, 0.0, 1.0)));

    glm::mat4 scale_mat = glm::transpose(glm::mat4(glm::vec4(_scale, 0.0, 0.0, 0.0),
                                                   glm::vec4(0.0, _scale, 0.0, 0.0),
                                                   glm::vec4(0.0, 0.0, _scale, 0.0),
                                                   glm::vec4(0.0, 0.0, 0.0, 1.0)));

    glm::mat4 transform = trans_mat * scale_mat;

    glPushMatrix();
    glMultMatrixf(glm::value_ptr(transform));
}

void FluidRenderer::_unsetTransforms() {
    glPopMatrix();
}

void FluidRenderer::_drawWireframeCube(glm::vec3 pos, double size) {
    double h = 0.5*size;
    glBegin(GL_LINES);
    glVertex3d(pos.x - h, pos.y - h, pos.z - h);
    glVertex3d(pos.x + h, pos.y - h, pos.z - h);
    glVertex3d(pos.x - h, pos.y - h, pos.z - h);
    glVertex3d(pos.x - h, pos.y + h, pos.z - h);
    glVertex3d(pos.x - h, pos.y - h, pos.z - h);
    glVertex3d(pos.x - h, pos.y - h, pos.z + h);

    glVertex3d(pos.x + h, pos.y + h, pos.z + h);
    glVertex3d(pos.x - h, pos.y + h, pos.z + h);
    glVertex3d(pos.x + h, pos.y + h, pos.z + h);
    glVertex3d(pos.x + h, pos.y - h, pos.z + h);
    glVertex3d(pos.x + h, pos.y + h, pos.z + h);
    glVertex3d(pos.x + h, pos.y + h, pos.z - h);

    glVertex3d(pos.x - h, pos.y + h, pos.z + h);
    glVertex3d(pos.x - h, pos.y - h, pos.z + h);
    glVertex3d(pos.x - h, pos.y + h, pos.z + h);
    glVertex3d(pos.x - h, pos.y + h, pos.z - h);

    glVertex3d(pos.x + h, pos.y - h, pos.z + h);
    glVertex3d(pos.x - h, pos.y - h, pos.z + h);
    glVertex3d(pos.x + h, pos.y - h, pos.z + h);
    glVertex3d(pos.x + h, pos.y - h, pos.z - h);

    glVertex3d(pos.x + h, pos.y + h, pos.z - h);
    glVertex3d(pos.x + h, pos.y - h, pos.z - h);
    glVertex3d(pos.x + h, pos.y + h, pos.z - h);
    glVertex3d(pos.x - h, pos.y + h, pos.z - h);

    glEnd();
}

void FluidRenderer::drawAirCells() {
    _drawFluidMaterialType(M_AIR);
}

void FluidRenderer::drawFluidCells() {
    _drawFluidMaterialType(M_FLUID);
}

void FluidRenderer::drawSolidCells() {
    _drawFluidMaterialType(M_SOLID);
}

void FluidRenderer::_drawFluidMaterialType(int mType) {
    int depth, height, width;
    _fluidsim->getGridDimensions(&width, &height, &depth);

    _setTransforms();

    glBegin(GL_POINTS);
    glm::vec3 p;
    double size = _fluidsim->getCellSize();
    for (int k = 0; k < depth; k++) {
        for (int j = 0; j < height; j++) {
            for (int i = 0; i < width; i++) {
                if (_fluidsim->getMaterial(i, j, k) == mType) {
                    double x, y, z;
                    Grid3d::GridIndexToCellCenter(i, j, k, size, &x, &y, &z);
                    p = glm::vec3(x, y, z);
                    glVertex3d(p.x, p.y, p.z);
                }
            }
        }
    }
    glEnd();

    _unsetTransforms();
}

void FluidRenderer::drawGridBoundingBox() {
    int i, j, k;
    double dx = _fluidsim->getCellSize();
    _fluidsim->getGridDimensions(&i, &j, &k);

    double hw = (double)i * dx * 0.5;
    double hh = (double)j * dx * 0.5;
    double hd = (double)k * dx * 0.5;

    glm::vec3 pos = glm::vec3(hw, hh, hd);

    _setTransforms();
    glBegin(GL_LINES);

    glVertex3d(pos.x - hw, pos.y - hh, pos.z - hd);
    glVertex3d(pos.x + hw, pos.y - hh, pos.z - hd);
    glVertex3d(pos.x - hw, pos.y - hh, pos.z - hd);
    glVertex3d(pos.x - hw, pos.y + hh, pos.z - hd);
    glVertex3d(pos.x - hw, pos.y - hh, pos.z - hd);
    glVertex3d(pos.x - hw, pos.y - hh, pos.z + hd);

    glVertex3d(pos.x + hw, pos.y + hh, pos.z + hd);
    glVertex3d(pos.x - hw, pos.y + hh, pos.z + hd);
    glVertex3d(pos.x + hw, pos.y + hh, pos.z + hd);
    glVertex3d(pos.x + hw, pos.y - hh, pos.z + hd);
    glVertex3d(pos.x + hw, pos.y + hh, pos.z + hd);
    glVertex3d(pos.x + hw, pos.y + hh, pos.z - hd);

    glVertex3d(pos.x - hw, pos.y + hh, pos.z + hd);
    glVertex3d(pos.x - hw, pos.y - hh, pos.z + hd);
    glVertex3d(pos.x - hw, pos.y + hh, pos.z + hd);
    glVertex3d(pos.x - hw, pos.y + hh, pos.z - hd);

    glVertex3d(pos.x + hw, pos.y - hh, pos.z + hd);
    glVertex3d(pos.x - hw, pos.y - hh, pos.z + hd);
    glVertex3d(pos.x + hw, pos.y - hh, pos.z + hd);
    glVertex3d(pos.x + hw, pos.y - hh, pos.z - hd);

    glVertex3d(pos.x + hw, pos.y + hh, pos.z - hd);
    glVertex3d(pos.x + hw, pos.y - hh, pos.z - hd);
    glVertex3d(pos.x + hw, pos.y + hh, pos.z - hd);
    glVertex3d(pos.x - hw, pos.y + hh, pos.z - hd);

    glEnd();
    _unsetTransforms();
}

void FluidRenderer::drawGrid() {
    int i, j, k;
    double dx = _fluidsim->getCellSize();
    _fluidsim->getGridDimensions(&i, &j, &k);

    double x_len = (double)i * dx;
    double y_len = (double)j * dx;
    double z_len = (double)k * dx;

    _setTransforms();
    glBegin(GL_LINES);

    for (int kk = 0; kk < k + 1; kk++) {
        for (int jj = 0; jj < j + 1; jj++) {
            double y = (double)jj * dx;
            double z = (double)kk * dx;
            glVertex3d(0.0, y, z);
            glVertex3d(x_len, y, z);
        }
    }

    for (int kk = 0; kk < k + 1; kk++) {
        for (int ii = 0; ii < i + 1; ii++) {
            double x = (double)ii * dx;
            double z = (double)kk * dx;
            glVertex3d(x, 0.0, z);
            glVertex3d(x, y_len, z);
        }
    }

    for (int jj = 0; jj < j + 1; jj++) {
        for (int ii = 0; ii < i + 1; ii++) {
            double x = (double)ii * dx;
            double y = (double)jj * dx;
            glVertex3d(x, y, 0.0);
            glVertex3d(x, y, z_len);
        }
    }

    glEnd();
    _unsetTransforms();

}

void FluidRenderer::drawMarkerParticles() {
    std::vector<glm::vec3> points = _fluidsim->getMarkerParticles(3);

    _setTransforms();

    glBegin(GL_POINTS);
    for (unsigned int i = 0; i < points.size(); i++) {
        (points[i].x, points[i].y, points[i].z);
    }
    glEnd();

    _unsetTransforms();
}

bool compareByDistance(const std::pair<glm::vec4, int> p1, std::pair<glm::vec4, int> p2) {
    return p1.first.w > p2.first.w;
}

void FluidRenderer::drawBillboardTextures(GLuint tex, double width, Camera3d *cam) {
    glm::vec3 cp = cam->getPosition();
    glm::vec3 cup = cam->up;
    double hw = 0.5f*width;

    glEnable(GL_TEXTURE_2D);
    glDisable(GL_LIGHTING);
    glBindTexture(GL_TEXTURE_2D, tex);

    std::vector<glm::vec3> points = _fluidsim->getMarkerParticles();
    std::vector<glm::vec3> solidpoints = _fluidsim->getSolidCellPositions();
    std::vector<DiffuseParticle> diffusePoints = _fluidsim->getDiffuseParticles();
    std::vector<std::pair<glm::vec4, int> > sortedPoints;

    glm::vec3 r;
    for (unsigned int i = 0; i < points.size(); i++) {
        glm::vec3 p = points[i];
        r = cp - p;
        double d = glm::dot(r, r);
        sortedPoints.push_back(std::pair<glm::vec4, int>(glm::vec4(p, d), 0));
    }
    for (unsigned int i = 0; i < solidpoints.size(); i++) {
        glm::vec3 p = solidpoints[i];
        r = cp - p;
        double d = glm::dot(r, r);
        sortedPoints.push_back(std::pair<glm::vec4, int>(glm::vec4(p, d), 1));
    }
    for (unsigned int i = 0; i < diffusePoints.size(); i++) {
        glm::vec3 p = diffusePoints[i].position;
        double type = diffusePoints[i].type;
        double t;
        r = cp - p;
        double d = glm::dot(r, r);

        if (type == 0) {
            t = 2;
        } else if (type == 1) {
            t = 3;
        } else if (type == 2) {
            t = 4;
        }

        sortedPoints.push_back(std::pair<glm::vec4, int>(glm::vec4(p, d), t));
    }

    std::sort(sortedPoints.begin(), sortedPoints.end(), compareByDistance);

    double e = 0.125;
    glm::vec3 eps = glm::vec3(e, e, e);
    glm::vec3 min = glm::vec3(0.125, 0.125, 0.125) + eps;
    glm::vec3 max = glm::vec3(0.125*63, 0.125*63, 0.125*63); - eps;
    AABB bbox = AABB(min, max);

    for (unsigned int i = 0; i < sortedPoints.size(); i++) {
        double size = hw;
        glm::vec4 p4 = sortedPoints[i].first;
        
        if (!bbox.isPointInside(glm::vec3(p4))) {
            continue;
        }

        int type = sortedPoints[i].second;
        glm::vec3 p = glm::vec3(p4.x, p4.y, p4.z);

        glm::vec3 look = glm::normalize(cp - p);
        glm::vec3 right = glm::normalize(glm::cross(cup, look));
        glm::vec3 up = glm::cross(look, right);

        glm::mat4 mat = glm::transpose(glm::mat4(right.x, up.x, look.x, p.x,
                                                 right.y, up.y, look.y, p.y,
                                                 right.z, up.z, look.z, p.z,
                                                 0.0, 0.0, 0.0, 1.0));
        glPushMatrix();
        glMultMatrixf((GLfloat*)&mat);

        if (type == 0) {                              // Fluid
            glColor4d(0.0, 0.753, 0.922, 1.0);
        } else if (type == 1) {                       // Solid
            glColor4d(0.5, 0.5, 0.5, 1.0);            
        } else if (type == 2) {                       // Bubble
            glColor4d(1.0, 0.0, 0.0, 1.0);
            size = hw/2.0;
        } else if (type == 3) {                       // Foam
            glColor4d(1.0, 1.0, 1.0, 1.0);
            size = hw/2.0;
        } else if (type == 4) {                       // Spray
            glColor4d(0.0, 1.0, 0.0, 1.0);
            size = hw/2.0;
        }

        glBegin(GL_QUADS);
        glTexCoord2f(0.0f, 0.0f); glVertex3d(-size, -size, 0.0f);
        glTexCoord2f(1.0f, 0.0f); glVertex3d(size, -size, 0.0f);
        glTexCoord2f(1.0f, 1.0f); glVertex3d(size, size, 0.0f);
        glTexCoord2f(0.0f, 1.0f); glVertex3d(-size, size, 0.0f);
        glEnd();

        glPopMatrix();
    }


    glEnable(GL_LIGHTING);
    glDisable(GL_TEXTURE_2D);

}

void FluidRenderer::drawLayerGrid() {
    Array3d<int> grid = _fluidsim->getLayerGrid();
    double size = _fluidsim->getCellSize();
    glm::vec3 p;

    _setTransforms();

    for (int k = 0; k < grid.depth; k++) {
        for (int j = 0; j < grid.height; j++) {
            for (int i = 0; i < grid.width; i++) {
                if (grid(i, j, k) > 0) {
                    double x, y, z;
                    Grid3d::GridIndexToCellCenter(i, j, k, size, &x, &y, &z);
                    p = glm::vec3(x, y, z);
                    _drawWireframeCube(p, 0.2*size);
                }
            }
        }
    }

    _unsetTransforms();
}

void FluidRenderer::drawSurfaceTriangles() {
    TriangleMesh *surface = _fluidsim->getFluidSurfaceTriangles();
    LevelSet *levelset = _fluidsim->getLevelSet();
    MACVelocityField *vfield = _fluidsim->getVelocityField();

    _setTransforms();
    glEnable(GL_NORMALIZE);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glShadeModel(GL_SMOOTH);
    glBegin(GL_TRIANGLES);

    float min = 0.35f;
    float max = 1.5f;

    glm::vec3 p1, p2, p3, n1, n2, n3;
    for (unsigned int i = 0; i < surface->triangles.size(); i++) {
        Triangle t = surface->triangles[i];
        p1 = surface->vertices[t.tri[0]];
        p2 = surface->vertices[t.tri[1]];
        p3 = surface->vertices[t.tri[2]];
        n1 = surface->normals[t.tri[0]];
        n2 = surface->normals[t.tri[1]];
        n3 = surface->normals[t.tri[2]];

        /*
        float k = levelset->getSurfaceCurvature(i);
        glm::vec3 p = (p1 + p2 + p3) / 3.0f;
        glm::vec3 n = glm::normalize((n1 + n2 + n3) / 3.0f);
        glm::vec3 v = glm::normalize(vfield->evaluateVelocityAtPosition(p));
        if (glm::dot(v, n) < 0.6) {
            k = 0.0f;
        }

        k = fmin(k, max);
        k = fmax(k, min);

        float f = (k - min) / (max - min);
        glm::vec3 c = (1.0f - f)*glm::vec3(0.8, 0.8, 0.8) + f*glm::vec3(0.0, 1.0, 0.0);
        glColor3d(c.x, c.y, c.z);
        */

        glNormal3f(n1.x, n1.y, n1.z);
        glVertex3d(p1.x, p1.y, p1.z);

        glNormal3f(n2.x, n2.y, n2.z);
        glVertex3d(p2.x, p2.y, p2.z);

        glNormal3f(n3.x, n3.y, n3.z);
        glVertex3d(p3.x, p3.y, p3.z);
    }
    glNormal3f(0.0, 0.0, 1.0); // glColor stops working if I don't do this?
    glEnd();

    glDisable(GL_NORMALIZE);
    _unsetTransforms();
}

void FluidRenderer::draw() {
}