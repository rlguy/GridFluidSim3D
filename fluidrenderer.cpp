#include "fluidrenderer.h"


FluidRenderer::FluidRenderer()
{
}

FluidRenderer::FluidRenderer(FluidSimulation &sim) : fluidsim(sim)
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
    fluidsim.getGridDimensions(&width, &height, &depth);

    _setTransforms();

    glm::vec3 p;
    double size = fluidsim.getCellSize();
    for (int k = 0; k < depth; k++) {
        for (int j = 0; j < height; j++) {
            for (int i = 0; i < width; i++) {
                if (fluidsim.getMaterial(i, j, k) == mType) {
                    double x, y, z;
                    fluidsim.gridIndexToCellCenter(i, j, k, &x, &y, &z);
                    p = glm::vec3(x, y, z);
                    _drawWireframeCube(p, 0.025);
                }
            }
        }
    }

    _unsetTransforms();
}

void FluidRenderer::drawGridBoundingBox() {
    int i, j, k;
    double dx = fluidsim.getCellSize();
    fluidsim.getGridDimensions(&i, &j, &k);

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
    double dx = fluidsim.getCellSize();
    fluidsim.getGridDimensions(&i, &j, &k);

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

void FluidRenderer::draw() {

}