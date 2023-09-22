#ifndef __EBO_H__
#define __EBO_H__

#include <glad/glad.h>
#include <vector>

class EBO
{
	unsigned int ID;
	unsigned int mCount;

public:
	// Constructor that generates a Elements Buffer Object and links it to indices
	EBO();
	EBO(const EBO& src): ID(src.ID) {}
	EBO(std::vector<unsigned int>& indices);
	~EBO();

	// Generates EBO ID
	void GenerateID();
	// Binds the EBO
	void Bind() const;
	void Bind(const std::vector<unsigned int>& indices) const;
	// Unbinds the EBO
	void Unbind() const;

	inline unsigned int getCount() const { return mCount; }
};

#endif