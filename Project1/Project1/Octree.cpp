#pragma once

#include "Octree.h"

void BoundingBox::SetRadius(float radius)
{
	m_radius = radius;
}

BoundingBox* OctreeNode::GetBoundingBox()
{
	return m_pBoundBox;
}



void OctreeNode::AddChildNode(OctreeNode* node)
{
	// �� ��忡 �ڽ� ��带 �߰��մϴ�.
	m_vChildren.push_back(node);
}

// �� ��带 �ø��մϴ�.
//void OctreeNode::CullNode(bool);          

 //�ڽ� ��带 �����ɴϴ�.
vector<OctreeNode*> const OctreeNode::GetChildNode(size_t)
{
	return m_vChildren;
}

//�θ� ��带 �����մϴ�.
void OctreeNode::SetParent(OctreeNode* const node) 
{
	m_pParent = node;
}

//�θ� ��带 ��ȯ�մϴ�.
OctreeNode* const OctreeNode::GetParent()
{
	return m_pParent;
}


//vCenter : ����� �߽� ��ġ   fHalfWidth : ����� ������     depthLimit : ���� ��� ���� �ܰ� 
OctreeNode* OctreeNode::BuildOctree(Vector3 vCenter, float fHalfWidth, int depthLimit) {

	//���ѵ� ���Դܰ迡 �����ϸ� �� �̻� �ڽ� ��带 �������� �ʽ��ϴ�.
	if (depthLimit < 0) {
		return NULL;
	}//if


	 //���� ��带 ����
	OctreeNode* pOctNode = new OctreeNode();
	BoundingBox* pBBox = pOctNode->GetBoundingBox();
	pOctNode->SetPosition(vCenter);
	pBBox->SetRadius(fHalfWidth);

	//��������� 8���� �ڽ� ������ �����մϴ�.
	Vector3 vOffset;
	Vector3 vChildCenter;
	float fStep = fHalfWidth * 0.5f;

	//8���� �ڽ� ���鿡 ���ؼ� �߽� ��ġ�� �����ϰ� Ʈ���� ����.
	for (int iTree = 0; iTree < 8; ++iTree) {

		vOffset[0] = ((iTree & 1) ? fStep : -fStep);
		vOffset[1] = ((iTree & 2) ? fStep : -fStep);
		vOffset[2] = ((iTree & 4) ? fStep : -fStep);

		vChildCenter[0] = vOffset[0] + vCenter[0];
		vChildCenter[1] = vOffset[1] + vCenter[1];
		vChildCenter[2] = vOffset[2] + vCenter[2];

		pOctNode->AddChildNode(BuildOctree(vChildCenter, fStep, depthLimit - 1));

	}//for
	return pOctNode;

}//VOID BuildOctree( FLOAT fCenter, FLOAT fHalfWidth,  size_t depth )


//vObjPos : �Ǵ��ϰ��� �ϴ� ��ü�� ��ġ           // pNode : �Ǵ��ϰ��� �ϴ� ���
bool  FindCurrentPosNode(const Vector3 vObjPos, OctreeNode* const pNode) {

	Vector3 pvNodePos = pNode->GetPosition();

	float fRadius = pNode->GetBoundingBox()->GetRadius();
	fRadius *= LOOSE_FACTOR;                                       // ������ ��Ʈ�� ���� ��� �������� ����� ������ �� �ֽ��ϴ�.

	float fMin, fMax;

	// �� �κ��� ���� ��ü�� �� ��� �ȿ� ������ ���ԵǴ� �� �Ǵ��մϴ�.
	for (int index = 0; index < 3; ++index) {
		fMin = (pvNodePos)[index] - fRadius;
		fMax = (pvNodePos)[index] + fRadius;

		//���Ե��� ������ ���и� ��ȯ����.
		if (vObjPos[index] < fMin || vObjPos[index] > fMax) {
			return false;
		}//if
	} //for

	 //���� ��� �ȿ� ��ü�� ������ ���Ѵٸ� �ش� ��带 ��ȯ  
	return true;


}//OctreeNode* FindCurrentPosNode( OctreeNode* const pNode )