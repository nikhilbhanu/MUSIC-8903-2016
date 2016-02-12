
#if !defined(__Dtw_HEADER_INCLUDED__)
#define __Dtw_HEADER_INCLUDED__

#pragma once

#include "ErrorDef.h"


class CDtw
{
public:
    enum MatrixDimension_t
    {
        kRow,
        kCol,

        kNumMatrixDimensions
    };

    CDtw(void);
    virtual ~CDtw(void);

    Error_t init (int iNumRows, int iNumCols);
    Error_t reset ();

    Error_t process (float **ppfDistanceMatrix);
 
    int getPathLength ();
    float getPathCost () const;
    Error_t getPath (int **ppiPathResult) const;

private:
    enum Directions_t
    {
        kHoriz,
        kVert,
        kDiag,

        kNumDirections
    };
    enum InternalMemory_t
    {
        kRowCurr,
        kRowNext,
        kColCurr,

        kNumVectors
    };

    static inline Directions_t findMinimum (float fHorizCost, float fVertCost, float fDiagCost, float &fResultCost)
    {
        Directions_t eDirection = kDiag;
        fResultCost             = fDiagCost;

        if (fHorizCost < fDiagCost)
        {
            eDirection  = kHoriz;
            fResultCost = fHorizCost;
        }
        if (fVertCost < fHorizCost && fVertCost < fDiagCost)
        {
            eDirection  = kVert;
            fResultCost = fVertCost;
        }

        return eDirection;
    }


    bool m_bIsInitialized;
    bool m_bWasProcessed;

    float *m_apfCost[kNumVectors];
    float m_fOverallCost;
    Directions_t  **m_ppePathIdx;
    int   m_iLengthOfPath;
    int   m_aiMatrixDimensions[kNumMatrixDimensions];
};


#endif
