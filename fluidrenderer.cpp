#include "fluidrenderer.h"


FluidRenderer::FluidRenderer()
{
}

FluidRenderer::FluidRenderer(FluidSimulation *sim) : fluidsim(sim)
{
}


FluidRenderer::~FluidRenderer()
{
}

void FluidRenderer::update(float dt) {

}

void FluidRenderer::_setTransforms() {
    glm::mat4 trans_mat = glm::transpose(glm::mat4(glm::vec4(1.0, 0.0, 0.0, tx),
                                                   glm::vec4(0.0, 1.0, 0.0, ty),
                                                   glm::vec4(0.0, 0.0, 1.0, tz),
                                                   glm::vec4(0.0, 0.0, 0.0, 1.0)));

    glm::mat4 scale_mat = glm::transpose(glm::mat4(glm::vec4(scale, 0.0, 0.0, 0.0),
                                                   glm::vec4(0.0, scale, 0.0, 0.0),
                                                   glm::vec4(0.0, 0.0, scale, 0.0),
                                                   glm::vec4(0.0, 0.0, 0.0, 1.0)));

    glm::mat4 transform = trans_mat * scale_mat;

    glPushMatrix();
    glMultMatrixf(glm::value_ptr(transform));
}

void FluidRenderer::_unsetTransforms() {
    glPopMatrix();
}

void FluidRenderer::_drawWireframeCube(glm::vec3 pos, double size) {
    float h = 0.5*size;
    glBegin(GL_LINES);
    glVertex3f(pos.x - h, pos.y - h, pos.z - h);
    glVertex3f(pos.x + h, pos.y - h, pos.z - h);
    glVertex3f(pos.x - h, pos.y - h, pos.z - h);
    glVertex3f(pos.x - h, pos.y + h, pos.z - h);
    glVertex3f(pos.x - h, pos.y - h, pos.z - h);
    glVertex3f(pos.x - h, pos.y - h, pos.z + h);

    glVertex3f(pos.x + h, pos.y + h, pos.z + h);
    glVertex3f(pos.x - h, pos.y + h, pos.z + h);
    glVertex3f(pos.x + h, pos.y + h, pos.z + h);
    glVertex3f(pos.x + h, pos.y - h, pos.z + h);
    glVertex3f(pos.x + h, pos.y + h, pos.z + h);
    glVertex3f(pos.x + h, pos.y + h, pos.z - h);

    glVertex3f(pos.x - h, pos.y + h, pos.z + h);
    glVertex3f(pos.x - h, pos.y - h, pos.z + h);
    glVertex3f(pos.x - h, pos.y + h, pos.z + h);
    glVertex3f(pos.x - h, pos.y + h, pos.z - h);

    glVertex3f(pos.x + h, pos.y - h, pos.z + h);
    glVertex3f(pos.x - h, pos.y - h, pos.z + h);
    glVertex3f(pos.x + h, pos.y - h, pos.z + h);
    glVertex3f(pos.x + h, pos.y - h, pos.z - h);

    glVertex3f(pos.x + h, pos.y + h, pos.z - h);
    glVertex3f(pos.x + h, pos.y - h, pos.z - h);
    glVertex3f(pos.x + h, pos.y + h, pos.z - h);
    glVertex3f(pos.x - h, pos.y + h, pos.z - h);

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
    fluidsim->getGridDimensions(&width, &height, &depth);

    _setTransforms();

    glBegin(GL_POINTS);
    glm::vec3 p;
    double size = fluidsim->getCellSize();
    for (int k = 0; k < depth; k++) {
        for (int j = 0; j < height; j++) {
            for (int i = 0; i < width; i++) {
                if (fluidsim->getMaterial(i, j, k) == mType) {
                    double x, y, z;
                    fluidsim->gridIndexToCellCenter(i, j, k, &x, &y, &z);
                    p = glm::vec3(x, y, z);
                    glVertex3f(p.x, p.y, p.z);
                }
            }
        }
    }
    glEnd();

    _unsetTransforms();
}

void FluidRenderer::drawGridBoundingBox() {
    int i, j, k;
    double dx = fluidsim->getCellSize();
    fluidsim->getGridDimensions(&i, &j, &k);

    double hw = (double)i * dx * 0.5;
    double hh = (double)j * dx * 0.5;
    double hd = (double)k * dx * 0.5;

    glm::vec3 pos = glm::vec3(hw, hh, hd);

    _setTransforms();
    glBegin(GL_LINES);

    glVertex3f(pos.x - hw, pos.y - hh, pos.z - hd);
    glVertex3f(pos.x + hw, pos.y - hh, pos.z - hd);
    glVertex3f(pos.x - hw, pos.y - hh, pos.z - hd);
    glVertex3f(pos.x - hw, pos.y + hh, pos.z - hd);
    glVertex3f(pos.x - hw, pos.y - hh, pos.z - hd);
    glVertex3f(pos.x - hw, pos.y - hh, pos.z + hd);

    glVertex3f(pos.x + hw, pos.y + hh, pos.z + hd);
    glVertex3f(pos.x - hw, pos.y + hh, pos.z + hd);
    glVertex3f(pos.x + hw, pos.y + hh, pos.z + hd);
    glVertex3f(pos.x + hw, pos.y - hh, pos.z + hd);
    glVertex3f(pos.x + hw, pos.y + hh, pos.z + hd);
    glVertex3f(pos.x + hw, pos.y + hh, pos.z - hd);

    glVertex3f(pos.x - hw, pos.y + hh, pos.z + hd);
    glVertex3f(pos.x - hw, pos.y - hh, pos.z + hd);
    glVertex3f(pos.x - hw, pos.y + hh, pos.z + hd);
    glVertex3f(pos.x - hw, pos.y + hh, pos.z - hd);

    glVertex3f(pos.x + hw, pos.y - hh, pos.z + hd);
    glVertex3f(pos.x - hw, pos.y - hh, pos.z + hd);
    glVertex3f(pos.x + hw, pos.y - hh, pos.z + hd);
    glVertex3f(pos.x + hw, pos.y - hh, pos.z - hd);

    glVertex3f(pos.x + hw, pos.y + hh, pos.z - hd);
    glVertex3f(pos.x + hw, pos.y - hh, pos.z - hd);
    glVertex3f(pos.x + hw, pos.y + hh, pos.z - hd);
    glVertex3f(pos.x - hw, pos.y + hh, pos.z - hd);

    glEnd();
    _unsetTransforms();
}

void FluidRenderer::drawGrid() {
    int i, j, k;
    double dx = fluidsim->getCellSize();
    fluidsim->getGridDimensions(&i, &j, &k);

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

void FluidRenderer::_drawImplicitPointData(ImplicitPointData point) {
    double r = point.radius;
    glm::vec3 p = point.position;

    glBegin(GL_LINES);
        glVertex3f(p.x - r, p.y, p.z);
        glVertex3f(p.x + r, p.y, p.z);
        glVertex3f(p.x, p.y - r, p.z);
        glVertex3f(p.x, p.y + r, p.z);
        glVertex3f(p.x, p.y, p.z - r);
        glVertex3f(p.x, p.y, p.z + r);
    glEnd();

    glBegin(GL_POINTS);
        glVertex3f(p.x, p.y, p.z);
        glVertex3f(p.x - r, p.y, p.z);
        glVertex3f(p.x + r, p.y, p.z);
        glVertex3f(p.x, p.y - r, p.z);
        glVertex3f(p.x, p.y + r, p.z);
        glVertex3f(p.x, p.y, p.z - r);
        glVertex3f(p.x, p.y, p.z + r);
    glEnd();
}

void FluidRenderer::drawImplicitFluidPoints() {
    std::vector<ImplicitPointData> points = fluidsim->getImplicitFluidPoints();

    _setTransforms();
    for (int i = 0; i < (int)points.size(); i++) {
        _drawImplicitPointData(points[i]);
    }
    _unsetTransforms();

}

void FluidRenderer::drawMarkerParticles() {
    std::vector<glm::vec3> points = fluidsim->getMarkerParticles(3);

    _setTransforms();

    glBegin(GL_POINTS);
    for (int i = 0; i < (int)points.size(); i++) {
        glVertex3f(points[i].x, points[i].y, points[i].z);
    }
    glEnd();

    _unsetTransforms();
}

bool compareByDistance(const std::pair<glm::vec4, bool> p1, std::pair<glm::vec4, bool> p2) {
    return p1.first.w > p2.first.w;
}

void FluidRenderer::drawBillboardTextures(GLuint tex, double width, Camera3d *cam) {
    glm::vec3 cp = cam->getPosition();
    glm::vec3 cup = cam->up;
    float hw = 0.5*width;

    glEnable(GL_TEXTURE_2D);
    glDisable(GL_LIGHTING);
    glBindTexture(GL_TEXTURE_2D, tex);

    std::vector<glm::vec3> points = fluidsim->getMarkerParticles();
    std::vector<glm::vec3> solidpoints = fluidsim->getSolidCellPositions();
    std::vector<std::pair<glm::vec4, bool> > sortedPoints;

    glm::vec3 r;
    for (int i = 0; i<points.size(); i++) {
        glm::vec3 p = points[i];
        r = cp - p;
        double d = glm::dot(r, r);
        sortedPoints.push_back(std::pair<glm::vec4, bool>(glm::vec4(p, d), true));
    }
    for (int i = 0; i<solidpoints.size(); i++) {
        glm::vec3 p = solidpoints[i];
        r = cp - p;
        double d = glm::dot(r, r);
        sortedPoints.push_back(std::pair<glm::vec4, bool>(glm::vec4(p, d), false));
    }

    std::sort(sortedPoints.begin(), sortedPoints.end(), compareByDistance);

    for (int i = 0; i < (int)sortedPoints.size(); i++) {
        glm::vec4 p4 = sortedPoints[i].first;
        bool isFluid = sortedPoints[i].second;
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

        if (isFluid) {
            glColor4f(0.0, 0.753, 0.922, 1.0);
        }
        else {
            glColor4f(0.5, 0.5, 0.5, 1.0);
        }

        glBegin(GL_QUADS);
        glTexCoord2f(0.0f, 0.0f); glVertex3f(-hw, -hw, 0.0f);
        glTexCoord2f(1.0f, 0.0f); glVertex3f(hw, -hw, 0.0f);
        glTexCoord2f(1.0f, 1.0f); glVertex3f(hw, hw, 0.0f);
        glTexCoord2f(0.0f, 1.0f); glVertex3f(-hw, hw, 0.0f);
        glEnd();

        glPopMatrix();
    }


    glEnable(GL_LIGHTING);
    glDisable(GL_TEXTURE_2D);

}

void FluidRenderer::drawLayerGrid() {
    Array3d<int> grid = fluidsim->getLayerGrid();
    double size = fluidsim->getCellSize();
    glm::vec3 p;

    _setTransforms();

    for (int k = 0; k < grid.depth; k++) {
        for (int j = 0; j < grid.height; j++) {
            for (int i = 0; i < grid.width; i++) {
                if (grid(i, j, k) > 0) {
                    double x, y, z;
                    fluidsim->gridIndexToCellCenter(i, j, k, &x, &y, &z);
                    p = glm::vec3(x, y, z);
                    _drawWireframeCube(p, 0.2*size);
                }
            }
        }
    }

    _unsetTransforms();
}

void FluidRenderer::drawSurfaceCells() {
    std::vector<GridIndex> cells = fluidsim->getFluidSurfaceCells();

    _setTransforms();

    for (int i = 0; i < cells.size(); i++) {
        GridIndex c = cells[i];
        double x, y, z;
        fluidsim->gridIndexToCellCenter(c.i, c.j, c.k, &x, &y, &z);
        glm::vec3 p = glm::vec3(x, y, z);
        _drawWireframeCube(p, 0.2*fluidsim->getCellSize());
    }

    _unsetTransforms();
}

void FluidRenderer::drawSurfaceTriangles() {
    TriangleMesh *surface = fluidsim->getFluidSurfaceTriangles();

    _setTransforms();
    glEnable(GL_NORMALIZE);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glShadeModel(GL_FLAT);
    glBegin(GL_TRIANGLES);

    glm::vec3 p1, p2, p3, n1, n2, n3;
    for (int i = 0; i < surface->triangles.size(); i++) {
        Triangle t = surface->triangles[i];
        p1 = surface->vertices[t.tri[0]];
        p2 = surface->vertices[t.tri[1]];
        p3 = surface->vertices[t.tri[2]];
        n1 = surface->normals[t.tri[0]];
        n2 = surface->normals[t.tri[1]];
        n3 = surface->normals[t.tri[2]];

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