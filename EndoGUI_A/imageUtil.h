//
// Elvis Chen
// chene@cs.queensu.ca
//
// Department of Computing and Information Science
// Queen's University, Kingston, Ontario, Canada
//
// Feb. 17, 2000
//

//
// Filename:  imageUtil.h
//

// implements utility functions for image class.  The following are included:
//
// output = rowShift(input, numRow)
// output = colShift(input, nomCol)
// output = circularShift(input, numCol, numRow)
//
// note that in circularShift, column-shifting takes place BEFORE
// row-shifting
//
//
// -- Convolution in 2D --
// the following performs circular convolution for 2D matrix
//
// output = convolve2D(input, mask)
// convolve2D(input, mask, output)
//

#ifndef __IMAGEUTIL_H__
#define __IMAGEUTIL_H__

template<class MAT>
MAT rowShift( const MAT &A, Subscript numRow ) 
{
  
  Subscript X = A.num_rows();
  Subscript Y = A.num_cols();
  
  Subscript i, j;
  
  MAT output( X, Y );
  
  numRow = numRow % X;
    
  if ( numRow == 0 ) {
    output = A;  // no shift is needed
  }
  else {
    
    if (numRow < 0) numRow += X;
    
    for (i = numRow; i < X; i++) {
      for (j = 0; j < Y; j++) {
	output[i][j] = A[i-numRow][j];
      }
    }
    
    for (i = 0; i < numRow; i++) {
      for (j = 0; j < Y; j++) {
	output[i][j] = A[ X - numRow + i ][j];
      }
    }
  }
  
  
  return (output);
  
}


//
// Column Shift
//
template<class MAT>
MAT colShift( const MAT &A, Subscript numCol ) 
{
  
  Subscript X = A.num_rows();
  Subscript Y = A.num_cols();
  
  Subscript i, j;
  
  MAT output( X, Y );
  
  numCol = numCol % Y;
    
  if ( numCol == 0 ) {
    output = A;  // no shift is needed
  }
  else {
    if ( numCol < 0 ) numCol += Y;
    
    for (i = numCol; i < Y; i++) {
      for (j = 0; j < X; j++) {
	output[j][i] = A[j][i-numCol];
      }
    }
    
    for (i = 0; i < numCol; i++) {
      for (j = 0; j < X; j++) {
	output[j][i] = A[j][ Y - numCol + i ];
      }
    }
    
  }
  
  
  return (output);
  
}


//
// Circular Shift
//
template<class MAT>
MAT circularShift( const MAT &A, Subscript numCol, Subscript numRow ) 
{
  //
  // in our version of circularShift, column shift take place
  // before row shift.
  //
  
  Subscript X = A.num_rows();
  Subscript Y = A.num_cols();
  
  MAT output( X, Y );
  
  numRow = numRow % X;
  numCol = numCol % Y;
  
  if ( (numRow == 0) && (numCol == 0) ) {
    output = A;
  }
  else if ( (numRow == 0) && !(numCol == 0) ) {
    output = colShift(A, numCol);
  }
  else if ( !(numRow == 0) && (numCol == 0) ) {
    output = rowShift(A, numRow);
  }
  else {

    // perform column shift BEFORE row shift, the same
    // as David Fleet.
    output = rowShift( colShift(A, numCol), numRow );
  }
    
  return (output);
  
}


//
// circular convolution in 2D
// 
template<class MAT>
MAT convolve2D( const MAT &A, const MAT &mask ) 
{
  
  Subscript inX = A.num_rows();
  Subscript inY = A.num_cols();

  Subscript maskX = mask.num_rows();
  Subscript maskY = mask.num_cols();
  
  Subscript x_shift = maskX / 2;
  Subscript y_shift = maskY / 2;
  
  Subscript x, y, X, Y, i, j;
  typename MAT::element_type sum;
  
  MAT outImg( inX, inY );
  
  for (y = 0; y < inY; y++) {
    for (x = 0; x < inX; x++) {
      sum = 0.0;
      
      for (i = 0; i < maskX; i++) {
	for (j = 0; j < maskY; j++) {
	  
	  X = x - j + x_shift;
	  Y = y - i + y_shift;
	  
	  if (X < 0 || X >= inX) X = ((X % inX) + inX) % inX;
	  if (Y < 0 || Y >= inY) Y = ((Y % inY) + inY) % inY;
	  
	  sum += A[Y][X] * mask[i][j];
	}
      }
      outImg[y][x] = sum;
    }
  }
  

  return( outImg );
}

//
// circular convolution in 2D
// 
template<class MAT>
inline void convolve2D( const MAT &A, const MAT &mask, MAT &outImg ) 
{
  outImg = convolve2D( A, mask );
}

#endif // of __IMAGEUTIL_H__
