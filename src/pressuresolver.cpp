/*
Copyright (c) 2016 Ryan L. Guy

This software is provided 'as-is', without any express or implied
warranty. In no event will the authors be held liable for any damages
arising from the use of this software.

Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not
   claim that you wrote the original software. If you use this software
   in a product, an acknowledgement in the product documentation would be
   appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be
   misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
#include "pressuresolver.h"

/********************************************************************************
    VectorXd
********************************************************************************/

VectorXd::VectorXd() {
}

VectorXd::VectorXd(int size) : _vector(size, 0.0) {
}

VectorXd::VectorXd(int size, double fill) : _vector(size, fill) {
}

VectorXd::VectorXd(VectorXd &vector) {
	_vector.reserve(vector.size());
	for (unsigned int i = 0; i < vector.size(); i++) {
		_vector.push_back(vector[i]);
	}
}

VectorXd::~VectorXd() {
}

const double VectorXd::operator[](int i) const {
    FLUIDSIM_ASSERT(i >= 0 && i < (int)_vector.size());
    return _vector[i];
}

double& VectorXd::operator[](int i) {
    FLUIDSIM_ASSERT(i >= 0 && i < (int)_vector.size());
    return _vector[i];
}

void VectorXd::fill(double fill) {
	for (unsigned int i = 0; i < _vector.size(); i++) {
		_vector[i] = fill;
	}
}

double VectorXd::dot(VectorXd &vector) {
	FLUIDSIM_ASSERT(_vector.size() == vector._vector.size());

	double sum = 0.0;
	for (unsigned int i = 0; i < _vector.size(); i++) {
		sum += _vector[i] * vector._vector[i];
	}

	return sum;
}

double VectorXd::absMaxCoeff() {
	double max = -std::numeric_limits<double>::infinity();
	for (unsigned int i = 0; i < _vector.size(); i++) {
		if (fabs(_vector[i]) > max) {
			max = fabs(_vector[i]);
		}
	}

	return max;
}

/********************************************************************************
    MatrixCoefficients
********************************************************************************/

MatrixCoefficients::MatrixCoefficients() {
}

MatrixCoefficients::MatrixCoefficients(int size) : cells(size, MatrixCell()) {
}

MatrixCoefficients::~MatrixCoefficients() {
}

const MatrixCell MatrixCoefficients::operator[](int i) const {
    FLUIDSIM_ASSERT(i >= 0 && i < (int)cells.size());
    return cells[i];
}

MatrixCell& MatrixCoefficients::operator[](int i) {
    FLUIDSIM_ASSERT(i >= 0 && i < (int)cells.size());
    return cells[i];
}

/********************************************************************************
    PressureSolver
********************************************************************************/

PressureSolver::PressureSolver() {
}

PressureSolver::~PressureSolver() {
}

void PressureSolver::solve(PressureSolverParameters params, VectorXd &pressure) {
	_initialize(params);

    FLUIDSIM_ASSERT(pressure.size() == (unsigned int)_matSize);
    pressure.fill(0.0);

	_initializeGridIndexKeyMap();

	VectorXd b(_matSize);
	_calculateNegativeDivergenceVector(b);
	if (b.absMaxCoeff() < _pressureSolveTolerance) {
		return;
	}

    MatrixCoefficients A(_matSize);
    _calculateMatrixCoefficients(A);

    VectorXd precon(_matSize);
    _calculatePreconditionerVector(A, precon);

    _solvePressureSystem(A, b, precon, pressure);
}

void PressureSolver::_initialize(PressureSolverParameters params) {
	_isize = params.materialGrid->width;
	_jsize = params.materialGrid->height;
	_ksize = params.materialGrid->depth;
	_dx = params.cellwidth;
	_density = params.density;
	_deltaTime = params.deltaTime;
	_fluidCells = params.fluidCells;
	_materialGrid = params.materialGrid;
	_vField = params.velocityField;
    _logfile = params.logfile;
	_matSize = (int)_fluidCells->size();


}

void PressureSolver::_initializeGridIndexKeyMap() {
	_keymap = GridIndexKeyMap(_isize, _jsize, _ksize);
	for (unsigned int idx = 0; idx < _fluidCells->size(); idx++) {
		_keymap.insert(_fluidCells->at(idx), idx);
	}
}

void PressureSolver::_calculateNegativeDivergenceVector(VectorXd &b) {

	double scale = 1.0f / (float)_dx;
    for (unsigned int idx = 0; idx < _fluidCells->size(); idx++) {
        int i = _fluidCells->at(idx).i;
        int j = _fluidCells->at(idx).j;
        int k = _fluidCells->at(idx).k;

        double value = -scale * (double)(_vField->U(i + 1, j, k) - _vField->U(i, j, k) +
                                         _vField->V(i, j + 1, k) - _vField->V(i, j, k) +
                                         _vField->W(i, j, k + 1) - _vField->W(i, j, k));
        
        b[_GridToVectorIndex(i, j, k)] = value;
    }

    // No functionality for moving solid cells, so velocity is 0
    float usolid = 0.0;
    float vsolid = 0.0;
    float wsolid = 0.0;
    for (unsigned int idx = 0; idx < _fluidCells->size(); idx++) {
        int i = _fluidCells->at(idx).i;
        int j = _fluidCells->at(idx).j;
        int k = _fluidCells->at(idx).k;
        int vidx = _GridToVectorIndex(i, j, k);

        if (_materialGrid->isCellSolid(i-1, j, k)) {
            b[vidx] -= (float)scale*(_vField->U(i, j, k) - usolid);
        }
        if (_materialGrid->isCellSolid(i+1, j, k)) {
            b[vidx] += (float)scale*(_vField->U(i+1, j, k) - usolid);
        }

        if (_materialGrid->isCellSolid(i, j-1, k)) {
            b[vidx] -= (float)scale*(_vField->V(i, j, k) - vsolid);
        }
        if (_materialGrid->isCellSolid(i, j+1, k)) {
            b[vidx] += (float)scale*(_vField->V(i, j+1, k) - vsolid);
        }

        if (_materialGrid->isCellSolid(i, j, k-1)) {
            b[vidx] -=  (float)scale*(_vField->W(i, j, k) - wsolid);
        }
        if (_materialGrid->isCellSolid(i, j, k+1)) {
            b[vidx] += (float)scale*(_vField->W(i, j, k+1) - wsolid);
        }
    }
}

int PressureSolver::_getNumFluidOrAirCellNeighbours(int i, int j, int k) {
    int n = 0;
    if (!_materialGrid->isCellSolid(i-1, j, k)) { n++; }
    if (!_materialGrid->isCellSolid(i+1, j, k)) { n++; }
    if (!_materialGrid->isCellSolid(i, j-1, k)) { n++; }
    if (!_materialGrid->isCellSolid(i, j+1, k)) { n++; }
    if (!_materialGrid->isCellSolid(i, j, k-1)) { n++; }
    if (!_materialGrid->isCellSolid(i, j, k+1)) { n++; }

    return n;
}

void PressureSolver::_calculateMatrixCoefficients(MatrixCoefficients &A) {
    for (unsigned int idx = 0; idx < _fluidCells->size(); idx++) {
        int i = _fluidCells->at(idx).i;
        int j = _fluidCells->at(idx).j;
        int k = _fluidCells->at(idx).k;
        int vidx = _GridToVectorIndex(i, j, k);

        int n = _getNumFluidOrAirCellNeighbours(i, j, k);
        A.cells[vidx].diag = (char)n;

        if (_materialGrid->isCellFluid(i + 1, j, k)) {
            A.cells[vidx].plusi = 0x01;
        }

        if (_materialGrid->isCellFluid(i, j + 1, k)) {
            A.cells[vidx].plusj = 0x01;
        }

        if (_materialGrid->isCellFluid(i, j, k + 1)) {
            A.cells[vidx].plusk = 0x01;
        }
    }
}

void PressureSolver::_calculatePreconditionerVector(MatrixCoefficients &A, VectorXd &precon) {
    FLUIDSIM_ASSERT(A.size() == precon.size());

    double scale = _deltaTime / (_density*_dx*_dx);
    double negscale = -scale;

    double tau = 0.97;      // Tuning constant
    double sigma = 0.25;    // safety constant
    for (unsigned int idx = 0; idx < _fluidCells->size(); idx++) {
        int i = _fluidCells->at(idx).i;
        int j = _fluidCells->at(idx).j;
        int k = _fluidCells->at(idx).k;
        int vidx = _GridToVectorIndex(i, j, k);

        int vidx_im1 = _keymap.find(i - 1, j, k);
        int vidx_jm1 = _keymap.find(i, j - 1, k);
        int vidx_km1 = _keymap.find(i, j, k - 1);

        double diag = (double)A[vidx].diag*scale;

        double plusi_im1 = vidx_im1 != -1 ? (double)A[vidx_im1].plusi * negscale : 0.0;
        double plusi_jm1 = vidx_jm1 != -1 ? (double)A[vidx_jm1].plusi * negscale : 0.0;
        double plusi_km1 = vidx_km1 != -1 ? (double)A[vidx_km1].plusi * negscale : 0.0;

        double plusj_im1 = vidx_im1 != -1 ? (double)A[vidx_im1].plusj * negscale : 0.0;
        double plusj_jm1 = vidx_jm1 != -1 ? (double)A[vidx_jm1].plusj * negscale : 0.0;
        double plusj_km1 = vidx_km1 != -1 ? (double)A[vidx_km1].plusj * negscale : 0.0;

        double plusk_im1 = vidx_im1 != -1 ? (double)A[vidx_im1].plusk * negscale : 0.0;
        double plusk_jm1 = vidx_jm1 != -1 ? (double)A[vidx_jm1].plusk * negscale : 0.0;
        double plusk_km1 = vidx_km1 != -1 ? (double)A[vidx_km1].plusk * negscale : 0.0;

        double precon_im1 = vidx_im1 != -1 ? precon[vidx_im1] : 0.0;
        double precon_jm1 = vidx_jm1 != -1 ? precon[vidx_jm1] : 0.0;
        double precon_km1 = vidx_km1 != -1 ? precon[vidx_km1] : 0.0;

        double v1 = plusi_im1 * precon_im1;
        double v2 = plusj_jm1 * precon_jm1;
        double v3 = plusk_km1 * precon_km1;
        double v4 = precon_im1 * precon_im1;
        double v5 = precon_jm1 * precon_jm1;
        double v6 = precon_km1 * precon_km1;
        
        double e = diag - v1*v1 - v2*v2 - v3*v3 - 
            tau*(plusi_im1*(plusj_im1 + plusk_im1)*v4 +
                 plusj_jm1*(plusi_jm1 + plusk_jm1)*v5 +
                 plusk_km1*(plusi_km1 + plusj_km1)*v6);

        if (e < sigma*diag) {
            e = diag;
        }

        if (fabs(e) > 10e-9) {
            precon[vidx] = 1.0 / sqrt(e);
        }
    }
}

void PressureSolver::_applyPreconditioner(MatrixCoefficients &A, 
                                          VectorXd &precon,
                                          VectorXd &residual,
                                          VectorXd &vect) {

    double scale = _deltaTime / (_density*_dx*_dx);
    double negscale = -scale;

    // Solve A*q = residual
    VectorXd q(_matSize);
    for (unsigned int idx = 0; idx < _fluidCells->size(); idx++) {
        int i = _fluidCells->at(idx).i;
        int j = _fluidCells->at(idx).j;
        int k = _fluidCells->at(idx).k;
        int vidx = _GridToVectorIndex(i, j, k);

        int vidx_im1 = _keymap.find(i - 1, j, k);
        int vidx_jm1 = _keymap.find(i, j - 1, k);
        int vidx_km1 = _keymap.find(i, j, k - 1);

        double plusi_im1 = 0.0;
        double precon_im1 = 0.0;
        double q_im1 = 0.0;
        if (vidx_im1 != -1) {
            plusi_im1  = (double)A[vidx_im1].plusi * negscale;
            precon_im1 = precon[vidx_im1];
            q_im1      = q[vidx_im1];
        }

        double plusj_jm1 = 0.0;
        double precon_jm1 = 0.0;
        double q_jm1 = 0.0;
        if (vidx_jm1 != -1) {
            plusj_jm1  = (double)A[vidx_jm1].plusj * negscale;
            precon_jm1 = precon[vidx_jm1];
            q_jm1      = q[vidx_jm1];
        }

        double plusk_km1 = 0.0;
        double precon_km1 = 0.0;
        double q_km1 = 0.0;
        if (vidx_km1 != -1) {
            plusk_km1  = (double)A[vidx_km1].plusk * negscale;
            precon_km1 = precon[vidx_km1];
            q_km1      = q[vidx_km1];
        }

        double t = residual[vidx] - plusi_im1 * precon_im1 * q_im1 -
                                    plusj_jm1 * precon_jm1 * q_jm1 -
                                    plusk_km1 * precon_km1 * q_km1;

        t = t*precon[vidx];
        q[vidx] = t;
    }

    // Solve transpose(A)*z = q
    for (int idx = (int)_fluidCells->size() - 1; idx >= 0; idx--) {
        int i = _fluidCells->at(idx).i;
        int j = _fluidCells->at(idx).j;
        int k = _fluidCells->at(idx).k;
        int vidx = _GridToVectorIndex(i, j, k);

        int vidx_ip1 = _keymap.find(i + 1, j, k);
        int vidx_jp1 = _keymap.find(i, j + 1, k);
        int vidx_kp1 = _keymap.find(i, j, k + 1);

        double vect_ip1 = vidx_ip1 != -1 ? vect[vidx_ip1] : 0.0;
        double vect_jp1 = vidx_jp1 != -1 ? vect[vidx_jp1] : 0.0;
        double vect_kp1 = vidx_kp1 != -1 ? vect[vidx_kp1] : 0.0;

        double plusi = (double)A[vidx].plusi * negscale;
        double plusj = (double)A[vidx].plusj * negscale;
        double plusk = (double)A[vidx].plusk * negscale;

        double preconval = precon[vidx];
        double t = q[vidx] - plusi * preconval * vect_ip1 -
                             plusj * preconval * vect_jp1 -
                             plusk * preconval * vect_kp1;

        t = t*preconval;
        vect[vidx] = t;
    }
}

void PressureSolver::_applyMatrix(MatrixCoefficients &A, VectorXd &x, VectorXd &result) {
    FLUIDSIM_ASSERT(A.size() == x.size() && x.size() == result.size());

    double scale = _deltaTime / (_density*_dx*_dx);
    double negscale = -scale;

    for (unsigned int idx = 0; idx < _fluidCells->size(); idx++) {
        int i = _fluidCells->at(idx).i;
        int j = _fluidCells->at(idx).j;
        int k = _fluidCells->at(idx).k;

        // val = dot product of column vector x and idxth row of matrix A
        double val = 0.0;
        int vidx = _GridToVectorIndex(i - 1, j, k);
        if (vidx != -1) { val += x._vector[vidx]; }

        vidx = _GridToVectorIndex(i + 1, j, k);
        if (vidx != -1) { val += x._vector[vidx]; }

        vidx = _GridToVectorIndex(i, j - 1, k);
        if (vidx != -1) { val += x._vector[vidx]; }

        vidx = _GridToVectorIndex(i, j + 1, k);
        if (vidx != -1) { val += x._vector[vidx]; }

        vidx = _GridToVectorIndex(i, j, k - 1);
        if (vidx != -1) { val += x._vector[vidx]; }

        vidx = _GridToVectorIndex(i, j, k + 1);
        if (vidx != -1) { val += x._vector[vidx]; }

        val *= negscale;

        vidx = _GridToVectorIndex(i, j, k);
        val += (double)A.cells[vidx].diag * scale * x._vector[vidx];

        result._vector[vidx] = val;
    }
}

// v1 += v2*scale
void PressureSolver::_addScaledVector(VectorXd &v1, VectorXd &v2, double scale) {
    FLUIDSIM_ASSERT(v1.size() == v2.size());
    for (unsigned int idx = 0; idx < v1.size(); idx++) {
        v1._vector[idx] += v2._vector[idx]*scale;
    }
}

// result = v1*s1 + v2*s2
void PressureSolver::_addScaledVectors(VectorXd &v1, double s1, 
                                       VectorXd &v2, double s2,
                                       VectorXd &result) {
    FLUIDSIM_ASSERT(v1.size() == v2.size() && v2.size() == result.size());
    for (unsigned int idx = 0; idx < v1.size(); idx++) {
        result._vector[idx] = v1._vector[idx]*s1 + v2._vector[idx]*s2;
    }
}

// Solve (A*pressure = b) with Modified Incomplete Cholesky 
// Conjugate Gradient method (MICCG(0))
void PressureSolver::_solvePressureSystem(MatrixCoefficients &A, 
                                          VectorXd &b, 
                                          VectorXd &precon,
                                          VectorXd &pressure) {

    double tol = _pressureSolveTolerance;
    if (b.absMaxCoeff() < tol) {
        return;
    }

    VectorXd residual(b);
    VectorXd auxillary(_matSize);
    _applyPreconditioner(A, precon, residual, auxillary);

    VectorXd search(auxillary);

    double alpha = 0.0;
    double beta = 0.0;
    double sigma = auxillary.dot(residual);
    double sigmaNew = 0.0;
    int iterationNumber = 0;

    while (iterationNumber < _maxCGIterations) {
        _applyMatrix(A, search, auxillary);
        alpha = sigma / auxillary.dot(search);
        _addScaledVector(pressure, search, alpha);
        _addScaledVector(residual, auxillary, -alpha);

        if (residual.absMaxCoeff() < tol) {
            _logfile->log("CG Iterations: ", iterationNumber, 1);
            return;
        }

        _applyPreconditioner(A, precon, residual, auxillary);
        sigmaNew = auxillary.dot(residual);
        beta = sigmaNew / sigma;
        _addScaledVectors(auxillary, 1.0, search, beta, search);
        sigma = sigmaNew;

        iterationNumber++;

        if (iterationNumber % 10 == 0) {
            std::ostringstream ss;
            ss << "\tIteration #: " << iterationNumber <<
                  "\tEstimated Error: " << residual.absMaxCoeff() << std::endl;
            _logfile->print(ss.str());
        }
    }

    _logfile->log("Iterations limit reached.\t Estimated error : ",
                  residual.absMaxCoeff(), 1);
}