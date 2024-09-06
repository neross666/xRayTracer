#pragma once

class Primitive
{
public:
	Primitive(int materialID) : m_materialID(materialID){

	}
	~Primitive() = default;

private:
	const int m_materialID;
};
