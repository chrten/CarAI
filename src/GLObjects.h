#pragma once

#include <glad/glad.h>

#include <glm/glm.hpp>

#include <string>
#include <vector>


namespace GL
{

class Buffer
{
public:

  Buffer(GLenum target);
  virtual ~Buffer();

  void gen();
  void bind();
  void unbind();

  // following functions call bind() automatically
  void setData(GLsizeiptr size, const void* data, GLenum usage = GL_STATIC_DRAW);
  void getData(GLintptr offset, GLsizeiptr size, void* dst);
  GLsizeiptr getSize();
  unsigned char* mapBuffer(GLenum access = GL_READ_WRITE);
  void unmapBuffer();



  GLuint id() const { return m_id; }
  bool valid() const { return m_id != 0; }

private:

  GLenum m_target;
  GLuint m_id;
};

class VertexBuffer : public Buffer
{
public:
  VertexBuffer() : Buffer(GL_ARRAY_BUFFER) {}
  virtual ~VertexBuffer() {}
};

class IndexBuffer : public Buffer
{
public:
  IndexBuffer() : Buffer(GL_ELEMENT_ARRAY_BUFFER) {}
  virtual ~IndexBuffer() {}
};



class Shader
{
public:
  Shader(GLenum type);
  virtual ~Shader();


  bool compileFromMemory(const char* source);
  bool compileFromFile(const char* filename);


  GLuint id() const { return m_id; }
  bool valid() const { return m_id != 0; }
  const std::string& source() const { return m_source; }

private:

  GLenum m_type;
  GLuint m_id;

  std::string m_source;
};

class VertexShader : public Shader
{
public:
  VertexShader() : Shader(GL_VERTEX_SHADER) {}
  virtual ~VertexShader(){}
};

class FragmentShader : public Shader
{
public:
  FragmentShader() : Shader(GL_FRAGMENT_SHADER) {}
  virtual ~FragmentShader() {}
};

class Program
{
public:
  Program();
  virtual ~Program();


  bool linkFromFile(const char* vertexShaderFile, const char* fragmentShaderFile);
  bool linkFromMemory(const std::vector<Shader*>& shaders);


  void use();
  void disable();

  GLint uniformLocation(const char* name) const;
  GLint attribLocation(const char* name) const;



  void setUniform4f(const char* name, const glm::vec4& v);
  void setUniformMatrix4f(const char* name, const glm::mat4& v, bool transpose = false);



  GLuint id() const { return m_id; }
  bool valid() const { return m_id != 0; }

private:

  GLuint m_id;
};



class Mesh
{
public:
  Mesh(const char* filename);
  virtual ~Mesh();

  void draw();

  int numVertices() const { return m_numVerts; }
  int numTriangles() const { return m_numTris; }

  int vertexStride() const { return m_vertexStride; }

  VertexBuffer* vertexBuffer() const { return m_vbo; }
  IndexBuffer* indexBuffer() const { return m_ibo; }

private:

  VertexBuffer* m_vbo;
  IndexBuffer* m_ibo;

  int m_numVerts;
  int m_numTris;

  int m_vertexStride;
};



}