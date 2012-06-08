#include "TGATexture.h"
#include <stdio.h>
#include<string.h>

// �R���X�g���N�^
CTGATextureHelper::CTGATextureHelper()
{
	image = NULL;

	m_iWidth = 0;
	m_iHeight = 0;

	m_iNumChannel = 0;
}

// �f�X�g���N�^
CTGATextureHelper::~CTGATextureHelper()
{
	Release();
}

// �������
void CTGATextureHelper::Release()
{
	if(image)
	{
		delete[] image;
		image = NULL;
	}
}

// �ǂݍ���
bool CTGATextureHelper::LoadTGA(char *filename)
{
	if(!filename)
		return false;

	FILE*	fp;
	unsigned char tgaHeader[12];	// TGA�̃w�b�_�[
	// �񈳏k��TGA�̃w�b�_�[
	unsigned char unCompressHeader[12] = {0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	unsigned char header[6];
	unsigned char bitCount;			// �s�N�Z��������̃r�b�g��
	int colorChannel;				// 1�`�����l���̃r�b�g��
	long tgaSize;					// �摜�T�C�Y�i�S�s�N�Z���́j

	fp = fopen( filename, "rb");
	if(!fp)
		return false;

	// TGA�̃w�b�_�[�̓ǂݍ���
	fread(tgaHeader, 1, sizeof(tgaHeader), fp);

	// �񈳏k��TGA��������Ȃ��̂Ŕ񈳏k��TGA�̃w�b�_�[�Ɣ�r
	if(memcmp(unCompressHeader, tgaHeader, sizeof(unCompressHeader)) != 0)
	{
		// ���k�t�H�[�}�b�g�Ȃ玸�s
		fclose(fp);
		return false;
	}

	// �C���[�W�̏��̓ǂݍ���
	fread(header, 1, sizeof(header), fp);

	m_iWidth = header[1] * 256 + header[0];
	m_iHeight = header[3] * 256 + header[2];

	bitCount = header[4];

	colorChannel = bitCount / 8;
	tgaSize = m_iWidth * m_iHeight * colorChannel;

	//
	image = new unsigned char[sizeof(unsigned char) * tgaSize];
	fread(image, sizeof(unsigned char), tgaSize, fp);

	// BGR�ŕ���ł��镨��RGB�ɕ��ёւ���
	for(long i=0;i<tgaSize;i++)
	{
		unsigned char buf = image[i];
		image[i] = image[i + 2];
		image[i + 2] = buf;
	}

	fclose(fp);
 
	if(colorChannel == 3) m_iNumChannel = 3;
	else m_iNumChannel = 4;

	return true;
}