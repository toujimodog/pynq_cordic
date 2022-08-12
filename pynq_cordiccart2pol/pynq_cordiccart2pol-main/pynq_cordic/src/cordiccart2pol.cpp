#include "cordiccart2pol.h"

//�ǶȲ��ұ�
ap_fixed<W, I, AP_RND, AP_WRAP, 1> cordic_phase[NO_ITER] = {
    45, 26.56505118, 14.03624347, 7.125016349,
    3.576334375, 1.789910608, 0.8951737102, 0.4476141709,
    0.2238105004, 0.1119056771, 0.05595289189, 0.02797645262,
    0.01398822714,0.006994113675, 0.003497056851, 0.001748528427  //...
};

void cordiccart2pol(data_t x, data_t y, data_t *r, data_t *theta){
#pragma HLS INTERFACE s_axilite port=x bundle=CTRL
#pragma HLS INTERFACE s_axilite port=y  bundle=CTRL
#pragma HLS INTERFACE s_axilite port=r  bundle=CTRL
#pragma HLS INTERFACE s_axilite port=theta  bundle=CTRL
#pragma HLS INTERFACE s_axilite port=return  bundle=CTRL

	int i,flag;
	//��λ��32λ������8λ���з��Ŷ�����
	ap_fixed<W, I, AP_RND, AP_WRAP, 1> current_x = x;
	ap_fixed<W, I, AP_RND, AP_WRAP, 1> temp_x,sigma;
	ap_fixed<W, I, AP_RND, AP_WRAP, 1> current_theta = 0;
	ap_fixed<W, I, AP_RND, AP_WRAP, 1> current_y = y;
	ap_fixed<W, I, AP_RND, AP_WRAP, 1> factor = 1.0;

	//xλ�ڸ�����ʱ���Ƚ�xȡ���������ٸ���y��������180����Ӽ�-��flag��־��
	if(current_x < 0){
		current_x=-current_x;
		if(current_y<0){
			flag=1;
		}else{
			flag=2;
		}
	}else{
		flag = 0;
	}

	for(i=0;i<NO_ITER;i++){
#pragma HLS LOOP_TRIPCOUNT min=1 max=16
//#pragma HLS pipeline
#pragma HLS UNROLL
		//sigma��current_y�����෴
		sigma = (current_y < 0) ? 1 : -1;
		temp_x = current_x;
		current_x = current_x - sigma*current_y*factor;
		current_y = current_y + sigma*temp_x*factor;
		current_theta = current_theta - sigma*cordic_phase[i];
		//����һλ��������һ���Ƕȵ���ת
		factor = factor >> 1;

		if(i==NO_ITER-1){
			if(flag==1){
				current_theta = -180-current_theta;
			}else if(flag==2){
				current_theta = 180-current_theta;
			}else{
				current_theta = current_theta ;
			}
		    //����������֮��Pn�����ƽӽ�x�ᣬ��ʱcurrent_y = 0����֪��ת�˦�
			//r=current_x*��cos0����r=current_x*k
			*r = (float)current_x*0.607252935;
			//���Ƕ�תΪ������
			*theta = (float)current_theta*0.0174533;
		}

	}

}
