
#include "Vector.h"

#include "Dtw.h"

CDtw::CDtw( void ) :
    m_bIsInitialized(false),
    m_fOverallCost(0),
    m_ppePathIdx(0),
    m_iLengthOfPath(0)
{
    CVector::setZero(m_apfCost, kNumMatrixDimensions);
    CVector::setZero(m_aiMatrixDimensions, kNumMatrixDimensions);

    reset();
}

CDtw::~CDtw( void )
{
    reset();
}

Error_t CDtw::init( int iNumRows, int iNumCols )
{
    if (iNumRows <= 0 || iNumCols <= 0)
        return kFunctionInvalidArgsError;

    reset();

    m_aiMatrixDimensions[kRow]  = iNumRows;
    m_aiMatrixDimensions[kCol]  = iNumCols;

    // allocate memory
    for (int k = 0; k < kNumMatrixDimensions; k++)
    {
        m_apfCost[k] = new float [m_aiMatrixDimensions[k]];
    }
    m_ppePathIdx    = new Directions_t* [m_aiMatrixDimensions[kCol]];
    for (int i = 0; i < m_aiMatrixDimensions[kRow]; i++)
        m_ppePathIdx[i]     = new Directions_t [m_aiMatrixDimensions[kCol]];

    m_bIsInitialized    = true;

    return kNoError;
}

Error_t CDtw::reset()
{
    m_bIsInitialized    = false;

    if (m_ppePathIdx)
    {
        for (int i = 0; i < m_aiMatrixDimensions[kRow]; i++)
        {
            delete [] m_ppePathIdx[i];
            m_ppePathIdx[i] = 0;
        }
    }
    delete [] m_ppePathIdx;
    m_ppePathIdx    = 0;

    for (int i = 0; i < kNumMatrixDimensions; i++)
    {
        delete [] m_apfCost[i];
        m_apfCost[i]    = 0;

        m_aiMatrixDimensions[i] = 0;

    }
    m_fOverallCost  = 0;
    m_iLengthOfPath = 0;

    return kNoError;
}

Error_t CDtw::process(float **ppfDistanceMatrix)
{
    if (!m_bIsInitialized)
        return kNotInitializedError;

    if (!ppfDistanceMatrix)
        return kFunctionInvalidArgsError;

    // initialize
    m_apfCost[kRow][0]  = ppfDistanceMatrix[0][0];
    m_apfCost[kCol][0]  = ppfDistanceMatrix[0][0];
    for (int i = 1; i < m_aiMatrixDimensions[kRow]; i++)
    {
        m_apfCost[kRow][i]  = m_apfCost[kRow][i-1] + ppfDistanceMatrix[i][0];
        m_ppePathIdx[i][0]  = kVert;
    }

    return kNoError;
}

int CDtw::getPathLength() const
{
    return 0;
}

float CDtw::getPathCost() const
{
    return 0.0F;
}

Error_t CDtw::getPath( int *piPathResult ) const
{
    if (!piPathResult)
        return kFunctionInvalidArgsError;

    return kNoError;
}
