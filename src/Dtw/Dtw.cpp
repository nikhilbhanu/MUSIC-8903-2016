
#include "Vector.h"
#include "Util.h"

#include "Dtw.h"

CDtw::CDtw( void ) :
    m_bIsInitialized(false),
    m_bWasProcessed(false),
    m_fOverallCost(0),
    m_ppePathIdx(0),
    m_iLengthOfPath(0)
{
    CVector::setZero(m_apfCost, kNumVectors);
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
    m_apfCost[kRowCurr] = new float [m_aiMatrixDimensions[kCol]];
    m_apfCost[kRowNext] = new float [m_aiMatrixDimensions[kCol]];
    m_apfCost[kColCurr] = new float [m_aiMatrixDimensions[kRow]];

    m_ppePathIdx    = new Directions_t* [m_aiMatrixDimensions[kRow]];
    for (int i = 0; i < m_aiMatrixDimensions[kRow]; i++)
        m_ppePathIdx[i]     = new Directions_t [m_aiMatrixDimensions[kCol]];

    m_bIsInitialized    = true;

    return kNoError;
}

Error_t CDtw::reset()
{
    m_bIsInitialized    = false;
    m_bWasProcessed     = false;

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

    for (int i = 0; i < kNumVectors; i++)
    {
        delete [] m_apfCost[i];
        m_apfCost[i]    = 0;
    }
    m_aiMatrixDimensions[kRow]  = 0;
    m_aiMatrixDimensions[kCol]  = 0;
    m_fOverallCost              = 0;
    m_iLengthOfPath             = 0;

    return kNoError;
}

Error_t CDtw::process(float **ppfDistanceMatrix)
{
    if (!m_bIsInitialized)
        return kNotInitializedError;

    if (!ppfDistanceMatrix)
        return kFunctionInvalidArgsError;

    // initialize
    CVectorFloat::setZero(m_apfCost[kRowNext], m_aiMatrixDimensions[kCol]);
    m_apfCost[kRowCurr][0]      = ppfDistanceMatrix[0][0];
    m_apfCost[kColCurr][0]      = ppfDistanceMatrix[0][0];
    m_ppePathIdx[0][0]          = kDiag;
    for (int j = 1; j < m_aiMatrixDimensions[kCol]; j++)
    {
        m_apfCost[kRowCurr][j]  = m_apfCost[kRowCurr][j-1] + ppfDistanceMatrix[0][j];
        m_ppePathIdx[0][j]      = kHoriz;
    }
    for (int i = 1; i < m_aiMatrixDimensions[kRow]; i++)
    {
        m_apfCost[kColCurr][i]  = m_apfCost[kColCurr][i-1] + ppfDistanceMatrix[i][0];
        m_ppePathIdx[i][0]      = kVert;
    }

    // compute cost 'matrix' and store backtracking path
    for (int i = 1; i < m_aiMatrixDimensions[kRow]; i++)
    {
        m_apfCost[kRowNext][0] = m_apfCost[kColCurr][i];
        for (int j = 1; j < m_aiMatrixDimensions[kCol]; j++)
        {
            float fTmpCost = 0;
            m_ppePathIdx[i][j]      = findMinimum(m_apfCost[kColCurr][i], m_apfCost[kRowCurr][j], m_apfCost[kRowCurr][j-1], fTmpCost);
            m_apfCost[kColCurr][i]  = fTmpCost + ppfDistanceMatrix[i][j];
            m_apfCost[kRowNext][j]  = m_apfCost[kColCurr][i];  
        }
        CUtil::swap(m_apfCost[kRowCurr], m_apfCost[kRowNext]);
    }

    m_bWasProcessed = true;

    return kNoError;
}

int CDtw::getPathLength()
{
    int i = m_aiMatrixDimensions[kRow]-1;
    int j = m_aiMatrixDimensions[kCol]-1;
    m_iLengthOfPath = 0;

    if (!m_bWasProcessed)
        return m_iLengthOfPath;

    while (i >= 0 && j >= 0)
    {
        switch(m_ppePathIdx[i][j])
        {
        case kDiag:
            i--;
            j--;
            break;
        case kHoriz:
            j--;
            break;
        case kVert:
            i--;
            break;
        }
        m_iLengthOfPath++;
    }
    
    return m_iLengthOfPath;
}

float CDtw::getPathCost() const
{
    return m_apfCost[kRowCurr][m_aiMatrixDimensions[kCol]-1];
}

Error_t CDtw::getPath( int **ppiPathResult ) const
{
    if (!ppiPathResult)
        return kFunctionInvalidArgsError;

    if (m_iLengthOfPath <= 0)
        return kFunctionIllegalCallError;

    int iIdx = m_iLengthOfPath - 1;

    ppiPathResult[kRow][0]      = 0;
    ppiPathResult[kCol][0]      = 0;
    ppiPathResult[kRow][iIdx]   = m_aiMatrixDimensions[kRow]-1;
    ppiPathResult[kCol][iIdx]   = m_aiMatrixDimensions[kCol]-1;

    while (iIdx > 0)
    {
        switch(m_ppePathIdx[ppiPathResult[kRow][iIdx]][ppiPathResult[kCol][iIdx]])
        {
        case kDiag:
            ppiPathResult[kRow][iIdx-1] = ppiPathResult[kRow][iIdx] - 1 ;
            ppiPathResult[kCol][iIdx-1] = ppiPathResult[kCol][iIdx] - 1 ;
            break;
        case kHoriz:
            ppiPathResult[kRow][iIdx-1] = ppiPathResult[kRow][iIdx];
            ppiPathResult[kCol][iIdx-1] = ppiPathResult[kCol][iIdx] - 1 ;
            break;
        case kVert:
            ppiPathResult[kRow][iIdx-1] = ppiPathResult[kRow][iIdx] - 1 ;
            ppiPathResult[kCol][iIdx-1] = ppiPathResult[kCol][iIdx];
            break;
        }
        iIdx--;
    }

    return kNoError;
}

