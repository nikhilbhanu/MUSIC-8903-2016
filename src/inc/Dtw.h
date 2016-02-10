
#if !defined(__Dtw_HEADER_INCLUDED__)
#define __Dtw_HEADER_INCLUDED__

#pragma once

#include "ErrorDef.h"


class CDtw
{
public:

    CDtw(void);
    virtual ~CDtw(void);

    Error_t init (int iNumRows, int iNumCols);
    Error_t reset ();

    Error_t process (float **ppfDistanceMatrix);
 
    int getPathLength () const;
    float getPathCost () const;
    Error_t getPath (int *piPathResult) const;

private:
    enum MatrixDimension_t
    {
        kRow,
        kCol,

        kNumMatrixDimensions
    };
    enum Directions_t
    {
        kHoriz,
        kVert,
        kDiag,

        kNumDirections
    };
    bool m_bIsInitialized;

    float *m_apfCost[kNumMatrixDimensions];
    float m_fOverallCost;
    Directions_t  **m_ppePathIdx;
    int   m_iLengthOfPath;
    int   m_aiMatrixDimensions[kNumMatrixDimensions];
};


#endif
