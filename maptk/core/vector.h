/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_VECTOR_H_
#define MAPTK_VECTOR_H_

#include <iostream>

namespace maptk
{

/// A representation of a vector
template <unsigned N, typename T>
class vector_
{
public:
  /// typedef for easier self reference
  typedef vector_<N,T> self;

  /// Constructor - does not initialize
  vector_<N,T>() {}

  /// Copy Constructor
  vector_<N,T>(const vector_<N,T>& other)
  {
    memcpy( data_, other.data_, sizeof(data_) );
  }

  /// Assignment operator
  vector_<N,T>& operator=(const vector_<N,T>& other)
  {
    memcpy( data_, other.data_, sizeof(data_) );
    return *this;
  }

  /// Return the i-th element
  T& operator[](unsigned int i) { return data_[i]; }

  /// Return the i-th element (const)
  const T& operator[](unsigned int i) const { return data_[i]; }

  /// Return a pointer to the contiguous block of memory
  T* data() { return data_; }

  /// Return a pointer to the contiguous block of memory
  T const* data() const { return data_; }



  /// Add a scalar in place
  vector_<N,T>& operator+=( T s ) { self::add( data_, s, data_ ); return *this; }

  /// Subract a scalr in place
  vector_<N,T>& operator-=( T s ) { self::sub( data_, s, data_ ); return *this; }

  /// Multiply a scalar in place
  vector_<N,T>& operator*=( T s ) { self::mul( data_, s, data_ ); return *this; }

  /// Divide by a scalar in place
  vector_<N,T>& operator/=( T s ) { self::div( data_, s, data_ ); return *this; }

  /// Add a vector in place
  vector_<N,T>& operator+=( const vector_<N,T>& v ) { self::add( data_, v.data_, data_ ); return *this; }

  /// Subract a vector in place
  vector_<N,T>& operator-=( const vector_<N,T>& v ) { self::sub( data_, v.data_, data_ ); return *this; }

  /// Negate operator
  vector_<N,T> operator-() const
  {
    vector_<N,T> result;
    self::sub( T(0), data_, result.data_ );
    return result;
  }

  /// Helper routines for arithmetic. N is the size, and is the
  /// template parameter.

  inline static void add( const T* a, const T* b, T* r )
  {
    for ( unsigned int i=0; i < N; ++i,++r,++a,++b )
      *r = *a + *b;
  }

  inline static void add( const T* a, T b, T* r )
  {
    for ( unsigned int i=0; i < N; ++i,++r,++a )
      *r = *a + b;
  }

  inline static void sub( const T* a, const T* b, T* r )
  {
    for ( unsigned int i=0; i < N; ++i,++r,++a,++b )
      *r = *a - *b;
  }

  inline static void sub( const T* a, T b, T* r )
  {
    for ( unsigned int i=0; i < N; ++i,++r,++a )
      *r = *a - b;
  }

  inline static void sub( T a, const T* b, T* r )
  {
    for ( unsigned int i=0; i < N; ++i,++r,++b )
      *r = a - *b;
  }

  inline static void mul( const T* a, const T* b, T* r )
  {
    for ( unsigned int i=0; i < N; ++i,++r,++a,++b )
      *r = *a * *b;
  }

  inline static void mul( const T* a, T b, T* r )
  {
    for ( unsigned int i=0; i < N; ++i,++r,++a )
      *r = *a * b;
  }

  inline static void div( const T* a, const T* b, T* r )
  {
    for ( unsigned int i=0; i < N; ++i,++r,++a,++b )
      *r = *a / *b;
  }

  inline static void div( const T* a, T b, T* r )
  {
    for ( unsigned int i=0; i < N; ++i,++r,++a )
      *r = *a / b;
  }

protected:
  T data_[N];
};


//: Compute the 2D cross product
// \relatesalso vector_
template <typename T>
inline T
cross_product(const vector_<2,T>& v1, const vector_<2,T>& v2)
{
  return v1[0] * v2[1] - v1[1] * v2[0];
}


/// Compute the 3D cross product
/// \relatesalso vector_
template <typename T>
inline vector_<3,T>
cross_product(const vector_<3,T>& v1, const vector_<3,T>& v2)
{
  vector_<3,T> result;
  result[0] = v1[1] * v2[2] - v1[2] * v2[1];
  result[1] = v1[2] * v2[0] - v1[0] * v2[2];
  result[2] = v1[0] * v2[1] - v1[1] * v2[0];
  return result;
}

/// A representation of a 2D vector.
/// This derived class exists to add convenience
/// constructors and accessors
template <typename T>
class vector_2_ : public vector_<2,T>
{
public:
  /// Default Constructor
  vector_2_<T> () {}

  /// Copy Constructor
  vector_2_<T> (const vector_2_<T>& other)
  : vector_<2,T>(other) {}

  /// Constructor from base class
  vector_2_<T> (const vector_<2,T>& base)
  : vector_<2,T>(base) {}

  /// Constructor for a 2D vector
  vector_2_<T>(const T& x, const T& y)
  {
    this->data_[0] = x;
    this->data_[1] = y;
  }

  /// Accessor for the X coordinate
  T& x() { return this->data_[0]; }
  /// Accessor for the X coordinate (const)
  const T& x() const { return this->data_[0]; }
  /// Accessor for the Y coordinate
  T& y() { return this->data_[1]; }
  /// Accessor for the Y coordinate (const)
  const T& y() const { return this->data_[1]; }
};


/// A representation of a 3D vector.
/// This derived class exists to add convenience
/// constructors and accessors
template <typename T>
class vector_3_ : public vector_<3,T>
{
public:
  /// Default Constructor
  vector_3_<T> () {}

  /// Copy Constructor
  vector_3_<T> (const vector_3_<T>& other)
  : vector_<3,T>(other) {}

  /// Constructor from base class
  vector_3_<T> (const vector_<3,T>& base)
  : vector_<3,T>(base) {}

  /// Constructor for a 3D vector
  vector_3_<T>(const T& x, const T& y, const T& z)
  {
    this->data_[0] = x;
    this->data_[1] = y;
    this->data_[2] = z;
  }

  /// Accessor for the X coordinate
  T& x() { return this->data_[0]; }
  /// Accessor for the X coordinate (const)
  const T& x() const { return this->data_[0]; }
  /// Accessor for the Y coordinate
  T& y() { return this->data_[1]; }
  /// Accessor for the Y coordinate (const)
  const T& y() const { return this->data_[1]; }
  /// Accessor for the Z coordinate
  T& z() { return this->data_[2]; }
  /// Accessor for the Z coordinate (const)
  const T& z() const { return this->data_[2]; }
};


/// A representation of a 4D vector.
/// This derived class exists to add convenience
/// constructors and accessors
template <typename T>
class vector_4_ : public vector_<4,T>
{
public:
  /// Default Constructor
  vector_4_<T> () {}

  /// Copy Constructor
  vector_4_<T> (const vector_4_<T>& other)
  : vector_<4,T>(other) {}

  /// Constructor from base class
  vector_4_<T> (const vector_<4,T>& base)
  : vector_<4,T>(base) {}

  /// Constructor for a 4D vector
  vector_4_<T>(const T& x, const T& y, const T& z, const T& w)
  {
    this->data_[0] = x;
    this->data_[1] = y;
    this->data_[2] = z;
    this->data_[3] = w;
  }

  /// Accessor for the X coordinate
  T& x() { return this->data_[0]; }
  /// Accessor for the X coordinate (const)
  const T& x() const { return this->data_[0]; }
  /// Accessor for the Y coordinate
  T& y() { return this->data_[1]; }
  /// Accessor for the Y coordinate (const)
  const T& y() const { return this->data_[1]; }
  /// Accessor for the Z coordinate
  T& z() { return this->data_[2]; }
  /// Accessor for the Z coordinate (const)
  const T& z() const { return this->data_[2]; }
  /// Accessor for the W coordinate
  T& w() { return this->data_[3]; }
  /// Accessor for the W coordinate (const)
  const T& w() const { return this->data_[3]; }
};

typedef vector_2_<double> vector_2d;
typedef vector_2_<float> vector_2f;
typedef vector_3_<double> vector_3d;
typedef vector_3_<float> vector_3f;
typedef vector_4_<double> vector_4d;
typedef vector_4_<float> vector_4f;

/// output stream operator for a vector
template <unsigned N, typename T>
std::ostream&  operator<<(std::ostream& s, const vector_<N,T>& v);

/// input stream operator for a vector
template <unsigned N, typename T>
std::istream&  operator>>(std::istream& s, vector_<N,T>& v);


} // end namespace maptk


#endif // MAPTK_VECTOR_H_
