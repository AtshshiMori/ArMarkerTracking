/********************************************************
Shader Class
GLSL�V�F�[�_���Ǘ����郉�C�u����
�Q�l�F
Modern OpenGL Tutorial http://www.opengl-tutorial.org/
���䌤����    http://marina.sys.wakayama-u.ac.jp/~tokoi/?date=20090827

Change 20160119:
�V�F�[�_�̃\�[�X�R�[�h������𒼐ړǂގd�l��ǉ�
*********************************************************/

#pragma once

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <GL/gl.h>
#include <GL/glut.h>

class Shader
{
protected:
	GLuint vertexShader, fragmentShader;        // �V�F�[�_�I�u�W�F�N�g
	const char *vertexFileName, *fragmentFileName;        // �V�F�[�_�t�@�C����
	void readShaderCompile(GLuint shader, const char *file); // .shader�̃R���p�C��
	void readInlineShaderCompile(GLuint shader, const char *source); // �V�F�[�_�̃C�����C���\�[�X���R���p�C��
	void link(GLuint prog);        // �R���p�C������shader�������N����
public:
	GLuint program;         // �V�F�[�_�v���O����
	Shader();               // �R���X�g���N�^
	~Shader();              // �f�X�g���N�^
	Shader &operator=(Shader &_s) // �R�s�[�R���X�g���N�^
	{
		initGLSL(_s.vertexFileName, _s.fragmentFileName);
		return *this;
	}
	// ������
	// �t���O�����g�V�F�[�_�[�̗L���ŕ�����
	void initGLSL(const char *vertexFile);
	void initGLSL(const char *vertexFile, const char *fragmentFile);
	void initInlineGLSL(const char *vertexSource, const char *fragmentSource);
	// �L����
	void enable() { glUseProgram(program); }
	void disable() { glUseProgram(0); }

};