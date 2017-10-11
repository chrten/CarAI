
#include "GLObjects.h"

#include <assimp/cimport.h>
#include <assimp/scene.h>
#include <assimp/postprocess.h>


#include <fstream>
#include <iostream>



namespace GL
{


Buffer::Buffer(GLenum target)
  : m_target(target), m_id(0)
{
}

Buffer::~Buffer()
{
  if (m_id)
    glDeleteBuffers(1, &m_id);
}

void Buffer::gen()
{
  glGenBuffers(1, &m_id);
}

void Buffer::bind()
{
  if (!valid())
    gen();

  glBindBuffer(m_target, m_id);
}

void Buffer::unbind()
{
  glBindBuffer(m_target, 0);
}

void Buffer::setData(GLsizeiptr size, const void* data, GLenum usage /*= GL_STATIC_DRAW*/)
{
  bind();
  if (valid())
    glBufferData(m_target, size, data, usage);
}

void Buffer::getData(GLintptr offset, GLsizeiptr size, void* dst)
{
  bind();
  if (valid())
    glGetBufferSubData(m_target, offset, size, dst);
}

GLsizeiptr Buffer::getSize()
{
  GLint64 size = 0;
  bind();
  if (valid())
    glGetBufferParameteri64v(m_target, GL_BUFFER_SIZE, &size);
  return static_cast<GLsizeiptr>(size);
}


unsigned char* Buffer::mapBuffer(GLenum access)
{
  unsigned char* p = 0;
  bind();
  if (valid())
    p = reinterpret_cast<unsigned char*>(glMapBuffer(m_target, access));
  return p;
}


void Buffer::unmapBuffer()
{
  glUnmapBuffer(m_target);
}

Shader::Shader(GLenum type)
  : m_type(type), m_id(0)
{

}

Shader::~Shader()
{
  if (m_id)
    glDeleteShader(m_id);
}

bool Shader::compileFromFile(const char* filename)
{
  std::ifstream file(filename, std::ifstream::binary);

  if (file.is_open())
  {
    file.seekg(0, file.end);
    size_t size = file.tellg();
    file.seekg(0, file.beg);

    std::vector<char> source(size + 1, 0);
    file.read(&source[0], size);
    file.close();

    return compileFromMemory(&source[0]);
  }
  else
    std::cout << "file not found: " << filename << std::endl;

  return false;
}

bool Shader::compileFromMemory(const char* source)
{
  if (m_id)
    glDeleteShader(m_id);

  m_source = source;
  GLint len = static_cast<int>(m_source.length());

  m_id = glCreateShader(m_type);
  glShaderSource(m_id, 1, &source, &len);
  glCompileShader(m_id);

  // check for compile errors
  GLint success = 0;
  glGetShaderiv(m_id, GL_COMPILE_STATUS, &success);
  if (!success) 
  {
    GLint loglen = 0;
    glGetShaderiv(m_id, GL_INFO_LOG_LENGTH, &loglen);

    std::vector<char> log(loglen);
    glGetShaderInfoLog(m_id, loglen, &loglen, &log[0]);

    std::cerr << &log[0] << std::endl;

    
    glDeleteShader(m_id);
    m_id = 0;

    return false;
  }

  return true;
}

Program::Program()
  : m_id(0)
{

}

Program::~Program()
{
  if (m_id)
    glDeleteProgram(m_id);
}

bool Program::linkFromFile(const char* vertexShaderFile, const char* fragmentShaderFile)
{
  VertexShader* vertex = new VertexShader();
  FragmentShader* frag = new FragmentShader();

  bool success = true;
  if (!vertex->compileFromFile(vertexShaderFile) ||
    !frag->compileFromFile(fragmentShaderFile))
    success = false;

  if (success)
  {
    std::vector<Shader*> shaders(2);
    shaders[0] = vertex;
    shaders[1] = frag;

    success = linkFromMemory(shaders);
  }
  
  delete vertex;
  delete frag;

  return success;
}

bool Program::linkFromMemory(const std::vector<Shader*>& shaders)
{
  if (m_id)
    glDeleteProgram(m_id);


  m_id = glCreateProgram();

  size_t n = shaders.size();
  for (size_t i = 0; i < n; ++i)
  {
    if (shaders[i])
      glAttachShader(m_id, shaders[i]->id());
  }
  glLinkProgram(m_id);

  // check linker error

  GLint status = GL_FALSE;
  glGetProgramiv(m_id, GL_LINK_STATUS, &status);
  if (!status) 
  {
    GLint loglen = 0;
    glGetProgramiv(m_id, GL_INFO_LOG_LENGTH, &loglen);

    std::vector<char> log(loglen);
    glGetProgramInfoLog(m_id, loglen, &loglen, &log[0]);

    std::cerr << &log[0] << std::endl;

    glDeleteProgram(m_id);
    m_id = 0;

    return false;
  }

  return true;
}

void Program::use()
{
  if (m_id)
    glUseProgram(m_id);
}

void Program::disable()
{
  glUseProgram(0);
}

GLint Program::uniformLocation(const char* name) const
{
  if (m_id)
    return glGetUniformLocation(m_id, name);
  return -1;
}

GLint Program::attribLocation(const char* name) const
{
  if (m_id)
    return glGetAttribLocation(m_id, name);
  return -1;
}




void Program::setUniform4f(const char* name, const glm::vec4& v)
{
  GLint loc = uniformLocation(name);
  if (loc >= 0)
    glUniform4f(loc, v.x, v.y, v.z, v.w);
}


void Program::setUniformMatrix4f(const char* name, const glm::mat4& v, bool transpose)
{
  GLint loc = uniformLocation(name);
  if (loc >= 0)
    glUniformMatrix4fv(loc, 1, GL_FALSE, (GLfloat*)&v);
}

Mesh::Mesh(const char* filename)
  : m_vbo(0), m_ibo(0), m_numVerts(0), m_numTris(0), m_vertexStride(0)
{
  const aiScene* scene = aiImportFile(filename, aiProcessPreset_TargetRealtime_Quality);

  for (unsigned int i = 0; i < scene->mNumMeshes; ++i)
  {
    m_numVerts += scene->mMeshes[i]->mNumVertices;
    m_numTris += scene->mMeshes[i]->mNumFaces;
  }

  std::vector<float> verts(m_numVerts * 3);
  std::vector<int> tris(m_numTris * 3);

  int offsetV = 0;
  int offsetT = 0;

  for (unsigned int i = 0; i < scene->mNumMeshes; ++i)
  {
    aiMesh* mesh = scene->mMeshes[i];

    for (unsigned int k = 0; k < mesh->mNumVertices; ++k)
    {
      for (int m = 0; m < 3; ++m)
        verts[(offsetV + k) * 3 + m] = mesh->mVertices[k][m];
    }

    for (unsigned int k = 0; k < mesh->mNumFaces; ++k)
    {
      for (int m = 0; m < 3; ++m)
        tris[offsetT++] = offsetV + static_cast<int>(mesh->mFaces[k].mIndices[m]);
    }

    offsetV += mesh->mNumVertices;
  }

  aiReleaseImport(scene);
  scene = 0;


  if (m_numVerts)
  {
    m_vertexStride = 12;
    m_vbo = new VertexBuffer();
    m_vbo->setData(m_numVerts * m_vertexStride, &verts[0]);
  }

  if (m_numTris)
  {
    m_ibo = new IndexBuffer();
    m_ibo->setData(m_numTris * 12, &tris[0]);
  }


  //glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}


Mesh::~Mesh()
{
  delete m_vbo;
  delete m_ibo;
}

void Mesh::draw()
{
  if (m_vbo)
  {
    m_vbo->bind();

    // vertex layout
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 12, 0); // pos


    if (m_ibo && m_numTris)
    {
      m_ibo->bind();
      glDrawElements(GL_TRIANGLES, m_numTris * 3, GL_UNSIGNED_INT, 0);
      m_ibo->unbind();
    }
    else
      glDrawArrays(GL_POINTS, 0, m_numVerts);

    m_vbo->unbind();

    glDisableVertexAttribArray(0);
  }
}

}
