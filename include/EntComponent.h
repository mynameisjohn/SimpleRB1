#pragma once

class EntComponent
{
	int m_iUniqueID;
public:
	EntComponent() :
		m_iUniqueID( -1 )
	{
	}
	inline void SetID( const int id )
	{
		m_iUniqueID = id;
	}
	inline int GetID() const
	{
		return m_iUniqueID;
	}
};