/********************************************************
Shader Class
GLSL�V�F�[�_���Ǘ����郉�C�u����
�Q�l�F
Modern OpenGL Tutorial http://www.opengl-tutorial.org/
���䌤����    http://marina.sys.wakayama-u.ac.jp/~tokoi/?date=20090827

Change 20160119:
�V�F�[�_�̃\�[�X�R�[�h������𒼐ړǂގd�l��ǉ�
*********************************************************/

#include "stdafx.h"
#include "Shader.h"

Shader::Shader()
{
}

Shader::~Shader()
{
}

void Shader::readShaderCompile(GLuint shader, const char *file)
{
	FILE *fp;
	char *buf;
	GLsizei size, len;
	GLint compiled;
	// �t�@�C�����J��
	fopen_s(&fp, file, "rb");
	if (!fp) printf("�t�@�C�����J�����Ƃ��ł��܂��� %s\n", file);
	//�t�@�C���̖����Ɉړ������݈ʒu�𓾂�
	fseek(fp, 0, SEEK_END);
	size = ftell(fp);//�t�@�C���T�C�Y���擾
					 // �t�@�C���T�C�Y�̃��������m��
	buf = (GLchar *)malloc(size);
	if (buf == NULL) {
		printf("���������m�ۂł��܂���ł��� \n");
	}
	// �t�@�C����擪����ǂݍ���
	fseek(fp, 0, SEEK_SET);
	fread(buf, 1, size, fp);
	//�V�F�[�_�I�u�W�F�N�g�Ƀv���O�������Z�b�g
	glShaderSource(shader, 1, (const GLchar **)&buf, &size);
	//�V�F�[�_�ǂݍ��ݗ̈�̉��
	free(buf);
	fclose(fp);
	// �V�F�[�_�̃R���p�C��
	glCompileShader(shader);
	glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);
	if (compiled == GL_FALSE) {
		printf("�R���p�C���ł��܂���ł���!!: %s \n ", file);
		glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &size);
		if (size > 0) {
			buf = (char *)malloc(size);
			glGetShaderInfoLog(shader, size, &len, buf);
			printf(buf);
			free(buf);
		}
	}
}

void Shader::readInlineShaderCompile(GLuint shader, const char *source)
{
	char *infolog;
	GLsizei size, infolen;
	GLint compiled;
	//printf(source);
	size = strlen(source);
	// �V�F�[�_�I�u�W�F�N�g�Ƀv���O�������Z�b�g
	glShaderSource(shader, 1, (const GLchar**)&source, &size);

	// �V�F�[�_�̃R���p�C��
	glCompileShader(shader);
	glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);
	if (compiled == GL_FALSE) {
		printf("�R���p�C���ł��܂���ł���!!: InlineShader \n ");
		glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &size);
		if (size > 0) {
			infolog = (char *)malloc(size);
			glGetShaderInfoLog(shader, size, &infolen, infolog);
			printf(infolog);
			free(infolog);
		}
	}
}

void Shader::link(GLuint prog)
{
	GLsizei size, len;
	GLint linked;
	char *infoLog;
	glLinkProgram(prog);
	glGetProgramiv(prog, GL_LINK_STATUS, &linked);
	if (linked == GL_FALSE) {
		printf("�����N�ł��܂���ł���!! \n");
		glGetProgramiv(prog, GL_INFO_LOG_LENGTH, &size);
		if (size > 0) {
			infoLog = (char *)malloc(size);
			glGetProgramInfoLog(prog, size, &len, infoLog);
			printf(infoLog);
			free(infoLog);
		}
	}
}

void Shader::initGLSL(const char *vertexFile)
{
	// GPU,OpenGL���
	printf("VENDOR= %s \n", glGetString(GL_VENDOR));
	printf("GPU= %s \n", glGetString(GL_RENDERER));
	printf("OpenGL= %s \n", glGetString(GL_VERSION));
	printf("GLSL= %s \n", glGetString(GL_SHADING_LANGUAGE_VERSION));
	//�V�F�[�_�[�I�u�W�F�N�g�̍쐬
	vertexShader = glCreateShader(GL_VERTEX_SHADER);
	//�V�F�[�_�[�̓ǂݍ��݂ƃR���p�C��
	readShaderCompile(vertexShader, vertexFile);
	// �V�F�[�_�v���O�����̍쐬
	program = glCreateProgram();
	// �V�F�[�_�I�u�W�F�N�g���V�F�[�_�v���O�����Ɋ֘A�t����
	glAttachShader(program, vertexShader);
	// �V�F�[�_�I�u�W�F�N�g�̍폜
	glDeleteShader(vertexShader);
	// �V�F�[�_�v���O�����̃����N
	link(program);
}

void Shader::initGLSL(const char *vertexFile, const char *fragmentFile)
{
	//glew�̏�����
	GLenum err = glewInit();
	if (err != GLEW_OK) {
		printf("Error: %s\n", glewGetErrorString(err));
	}
	memcpy(&vertexFileName, &vertexFile, sizeof(vertexFile) / sizeof(char));
	memcpy(&fragmentFileName, &fragmentFile, sizeof(fragmentFile) / sizeof(char));
	// GPU,OpenGL���
	printf("VENDOR= %s \n", glGetString(GL_VENDOR));
	printf("GPU= %s \n", glGetString(GL_RENDERER));
	printf("OpenGL= %s \n", glGetString(GL_VERSION));
	printf("GLSL= %s \n", glGetString(GL_SHADING_LANGUAGE_VERSION));
	//�V�F�[�_�[�I�u�W�F�N�g�̍쐬
	vertexShader = glCreateShader(GL_VERTEX_SHADER);
	fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
	//�V�F�[�_�[�̓ǂݍ��݂ƃR���p�C��
	readShaderCompile(vertexShader, vertexFile);
	readShaderCompile(fragmentShader, fragmentFile);
	// �v���O�����I�u�W�F�N�g�̍쐬
	program = glCreateProgram();
	// �V�F�[�_�I�u�W�F�N�g���V�F�[�_�v���O�����Ɋ֘A�t����
	glAttachShader(program, vertexShader);
	glAttachShader(program, fragmentShader);
	// �V�F�[�_�I�u�W�F�N�g�̍폜
	glDeleteShader(vertexShader);
	glDeleteShader(fragmentShader);
	// �V�F�[�_�v���O�����̃����N
	link(program);
}

void Shader::initInlineGLSL(const char *vertexSource, const char *fragmentSource)
{
	//glew�̏�����
	GLenum err = glewInit();
	if (err != GLEW_OK) {
		printf("Error: %s\n", glewGetErrorString(err));
	}
	// �[���I�ɃV�F�[�_�t�@�C�������쐬
	char vertexFile[] = "InlineVertex", fragmentFile[] = "InlineFragment";
	memcpy(&vertexFileName, &vertexFile, sizeof(vertexFile) / sizeof(char));
	memcpy(&fragmentFileName, &fragmentFile, sizeof(fragmentFile) / sizeof(char));
	// GPU,OpenGL���
	printf("VENDOR= %s \n", glGetString(GL_VENDOR));
	printf("GPU= %s \n", glGetString(GL_RENDERER));
	printf("OpenGL= %s \n", glGetString(GL_VERSION));
	printf("GLSL= %s \n", glGetString(GL_SHADING_LANGUAGE_VERSION));
	//�V�F�[�_�[�I�u�W�F�N�g�̍쐬
	vertexShader = glCreateShader(GL_VERTEX_SHADER);
	fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
	//�V�F�[�_�[�̓ǂݍ��݂ƃR���p�C��
	readInlineShaderCompile(vertexShader, vertexSource);
	readInlineShaderCompile(fragmentShader, fragmentSource);
	// �v���O�����I�u�W�F�N�g�̍쐬
	program = glCreateProgram();
	// �V�F�[�_�I�u�W�F�N�g���V�F�[�_�v���O�����Ɋ֘A�t����
	glAttachShader(program, vertexShader);
	glAttachShader(program, fragmentShader);
	// �V�F�[�_�I�u�W�F�N�g�̍폜
	glDeleteShader(vertexShader);
	glDeleteShader(fragmentShader);
	// �V�F�[�_�v���O�����̃����N
	link(program);
}