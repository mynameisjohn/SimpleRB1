#pragma once

#include <chrono>
#include <string>
#include <iostream>
#include <algorithm>

// Smol
const float kEPS = 0.001f;
const float g_fTimeStep = 0.005f;
const float g_fInvTimeStep = 1.f / 0.005f;

// remaps x : [m0, M0] to the range of [m1, M1]
inline float remap( float x, float m0, float M0, float m1, float M1 )
{
	return m1 + ((x - m0) / (M0 - m0)) * (M1 - m1);
}

template<typename T>
T clamp( T x, T min, T max )
{
	return std::min( std::max( x, min ), max );
}

bool feq( float a, float b, float diff = kEPS );

// Always useful
using Time = std::chrono::high_resolution_clock;
class StopWatch
{
	decltype(Time::now()) m_Begin, m_End;
	std::string m_Name;
public:
	StopWatch( std::string s ) :
		m_Begin( Time::now() ),
		m_Name( s )
	{
	}
	~StopWatch()
	{
		using std::chrono::duration_cast;
		using std::chrono::milliseconds;
		m_End = Time::now();
		std::cout << m_Name << " took " << duration_cast<milliseconds>(m_End - m_Begin).count() << " mS";
	}
};

// Const ptr type
template<typename T>
using const_ptr = const T * const;

// Used for hashing pairs
template<typename P>
struct pair_hash
{
	template <class T>
	static inline void hash_combine( std::size_t & seed, const T& v )
	{
		std::hash<T> hasher;
		seed ^= hasher( v ) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
	}

	size_t operator()( const P& p ) const
	{
		std::size_t seed1 = 0;
		hash_combine( seed1, p.first );
		hash_combine( seed1, p.second );

		std::size_t seed2 = 0;
		hash_combine( seed2, p.second );
		hash_combine( seed2, p.first );

		size_t ret = std::min( seed1, seed2 );
		return ret;
	}
};

template <typename P>
struct pair_hash_eq
{
	bool operator()( const P& a, const P& b ) const
	{
		return pair_hash<P>()(a) == pair_hash<P>()(b);
	}
};