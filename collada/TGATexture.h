#ifndef _TGATEXTURE_H_
#define _TGATEXTURE_H_

// TGA���e�N�X�`���Ƃ��ė��p���邽�߂̃w���p�[�N���X
class CTGATextureHelper
{
    int m_iNumChannel;			// 1�s�N�Z���̐F�`�����l����(RGB or RGBA)

	// ���A����
	unsigned int m_iWidth;
    unsigned int m_iHeight;

	unsigned char *image;
	
public:
	CTGATextureHelper();
	~CTGATextureHelper();

	bool LoadTGA( char* filename );
	void Release();

	// ���̎擾
	unsigned int GetWidth(){ return m_iWidth; }
	// �����̎擾
	unsigned int GetHeight(){ return m_iHeight; }
	// �C���[�W�̃r�b�g��
	unsigned char* GetImage(){ return image; }

};

#endif