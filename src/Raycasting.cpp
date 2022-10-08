//#include""
//
//bool Ray::intersects(const AABB& aabb) const
//{
//	Vec3 ptOnPlane; //�������Χ��ĳ��Ľ���
//	Vec3 min = aabb._min; //aabb��Χ����С������
//	Vec3 max = aabb._max; //aabb��Χ����������
//
//	const Vec3& origin = _origin; //������ʼ��
//	const Vec3& dir = _direction; //����ʸ��
//
//	float t;
//
//	//�ֱ��ж������������ཻ���
//
//	//�ж��������Χ��x�᷽������Ƿ��н���
//	if (dir.x != 0.f) //����x�᷽�������Ϊ0 �����߷���ʸ����x�����Ϊ0�����߲����ܾ�����Χ�г�x�᷽���������
//	{
//		/*
//		ʹ��������ƽ���ཻ�Ĺ�ʽ�󽻵�
//		*/
//		if (dir.x > 0)//��������x��������ƫ��
//			t = (min.x - origin.x) / dir.x;
//		else  //������x�Ḻ����ƫ��
//			t = (max.x - origin.x) / dir.x;
//
//		if (t > 0.f) //t>0ʱ��������ƽ���ཻ
//		{
//			ptOnPlane = origin + t * dir; //���㽻������
//			//�жϽ����Ƿ��ڵ�ǰ����
//			if (min.y < ptOnPlane.y && ptOnPlane.y < max.y && min.z < ptOnPlane.z && ptOnPlane.z < max.z)
//			{
//				return true; //�������Χ���н���
//			}
//		}
//	}
//
//	//��������y�᷽���з��� �ж��Ƿ����Χ��y�᷽���н���
//	if (dir.y != 0.f)
//	{
//		if (dir.y > 0)
//			t = (min.y - origin.y) / dir.y;
//		else
//			t = (max.y - origin.y) / dir.y;
//
//		if (t > 0.f)
//		{
//			ptOnPlane = origin + t * dir;
//
//			if (min.z < ptOnPlane.z && ptOnPlane.z < max.z && min.x < ptOnPlane.x && ptOnPlane.x < max.x)
//			{
//				return true;
//			}
//		}
//	}
//
//	//��������z�᷽���з��� �ж��Ƿ����Χ��y�᷽���н���
//	if (dir.z != 0.f)
//	{
//		if (dir.z > 0)
//			t = (min.z - origin.z) / dir.z;
//		else
//			t = (max.z - origin.z) / dir.z;
//
//		if (t > 0.f)
//		{
//			ptOnPlane = origin + t * dir;
//
//			if (min.x < ptOnPlane.x && ptOnPlane.x < max.x && min.y < ptOnPlane.y && ptOnPlane.y < max.y)
//			{
//				return true;
//			}
//		}
//	}
//
//	return false;
//}
//
//
//bool BBox::hit(const Ray& ray) const
//{
//	double ox = ray.o.x; double oy = ray.o.y; double oz = ray.o.z;
//	double dx = ray.d.x; double dy = ray.d.y; double dz = ray.d.z;
//	double tx_min, ty_min, tz_min;
//	double tx_max, ty_max, tz_max;
//
//	//x0,y0,z0Ϊ��Χ�����С����
//	//x1,y1,z1Ϊ��Χ�����󶥵�
//	if (abs(dx) < 0.000001f)
//	{
//		//�����߷���ʸ����x�����Ϊ0��ԭ�㲻�ں�����
//		if (ox < x1 || ox > x0)
//			return false;
//	}
//	else
//	{
//		if (dx >= 0)
//		{
//			tx_min = (x0 - ox) / dx;
//			tx_max = (x1 - ox) / dx;
//		}
//		else
//		{
//			tx_min = (x1 - ox) / dx;
//			tx_max = (x0 - ox) / dx;
//		}
//
//	}
//
//
//	if (abs(dy) < 0.000001f)
//	{
//		//�����߷���ʸ����x�����Ϊ0��ԭ�㲻�ں�����
//		if (oy < y1 || oy > y0)
//			return false;
//	}
//	else
//	{
//		if (dy >= 0)
//		{
//			ty_min = (y0 - oy) / dy;
//			ty_max = (y1 - oy) / dy;
//		}
//		else
//		{
//			ty_min = (y1 - oy) / dy;
//			ty_max = (y0 - oy) / dy;
//		}
//
//	}
//
//
//	if (abs(dz) < 0.000001f)
//	{
//		//�����߷���ʸ����x�����Ϊ0��ԭ�㲻�ں�����
//		if (oz < z1 || oz > z0)
//			return false;
//	}
//	else
//	{
//		if (dz >= 0)
//		{
//			tz_min = (z0 - oz) / dz;
//			tz_max = (z1 - oz) / dz;
//		}
//		else
//		{
//			tz_min = (z1 - oz) / dz;
//			tz_max = (z0 - oz) / dz;
//		}
//
//	}
//
//	double t0, t1;
//
//	//���߽���ƽ�洦�������ƽ�棩�����tֵ 
//	t0 = max(tz_min, max(tx_min, ty_min));
//
//	//�����뿪ƽ�洦����Զ���ƽ�棩����Сtֵ
//	t1 = min(tz_max, min(tx_max, ty_max));
//
//	return t0<t1;
//}