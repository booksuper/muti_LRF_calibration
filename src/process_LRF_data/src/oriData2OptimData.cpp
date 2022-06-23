#include "oriData2OptimData.h"

/*
 *Author : ShuDengdeng
 *Email  : 2237380450@qq.com
 *功能：将拟合的直线方程转为优化器需要的数据
 *2022.623
*/

CO_vector CO2vector(const CO &co)
{
   CO_vector co_vector;
    for(size_t LRF_id=0; LRF_id < 2; LRF_id++) // 2 LRFs per CO
    {
        //co_vector前25个数字代表LRF1的数据
        //这一行就是设置LRF1和LRF2的分界
        co_vector(25*LRF_id,0) = co[LRF_id].id_LRF;
        //plane_id代表每个LRF观测的两条直线ID
        for(size_t plane_id=0; plane_id < 2; plane_id++) // 2 line observations per LRF
        {
            size_t pos_block = 25*LRF_id + 12*plane_id + 1;
            //将第一个LRF的第一条直线的方向向量存储在co_vector的前两个数字
            co_vector.block(pos_block,0,2,1) = co[LRF_id].lines_co[plane_id].dir;
            //同理，将方向向量的协方差矩阵按照一行一行存在3-6位置
            co_vector.block(pos_block+2,0,2,1) = co[LRF_id].lines_co[plane_id].cov_dir.block(0,0,2,1);
            co_vector.block(pos_block+4,0,2,1) = co[LRF_id].lines_co[plane_id].cov_dir.block(0,1,2,1);
            //      co_vector.block(pos_block+2,0,4,1) = Map<Matrix<double,4,1>(co[LRF_id].lines[plane_id].cov_center);
            //将直线的中心存在7-8
            co_vector.block(pos_block+6,0,2,1) = co[LRF_id].lines_co[plane_id].center;
            //将直线的中心协方差存在9-12
            co_vector.block(pos_block+8,0,2,1) = co[LRF_id].lines_co[plane_id].cov_center.block(0,0,2,1);
            co_vector.block(pos_block+10,0,2,1) = co[LRF_id].lines_co[plane_id].cov_center.block(0,1,2,1);
        }
    }

    return co_vector; 
}

//将co_vector的向量形式转回co
CO vector2CO(const CO_vector &co_vector)
{
    CO co;
    for(size_t LRF_id=0; LRF_id < 2; LRF_id++) // 2 LRFs per CO
    {
        co[LRF_id].id_LRF = co_vector(25*LRF_id,0);
        for(size_t plane_id=0; plane_id < 2; plane_id++) // 2 line observations per LRF
        {
            size_t pos_block = 25*LRF_id + 12*plane_id + 1;
            co[LRF_id].lines_co[plane_id].dir = co_vector.block(pos_block,0,2,1);
            co[LRF_id].lines_co[plane_id].cov_dir.block(0,0,2,1) = co_vector.block(pos_block+2,0,2,1);
            co[LRF_id].lines_co[plane_id].cov_dir.block(0,1,2,1) = co_vector.block(pos_block+4,0,2,1);
            co[LRF_id].lines_co[plane_id].center = co_vector.block(pos_block+6,0,2,1);
            co[LRF_id].lines_co[plane_id].cov_center.block(0,0,2,1) = co_vector.block(pos_block+8,0,2,1);
            co[LRF_id].lines_co[plane_id].cov_center.block(0,1,2,1) = co_vector.block(pos_block+10,0,2,1);
        }
    }

    return co;
}

//生成cos数据
void generate_cos(int M_num_LRFs,vector<vector<pair<size_t,TLine2D_My> > > detected_lines,vector<CO> &vCOs)
{
     for(int j=0; j < M_num_LRFs; j++)
                if(detected_lines[j].size() > 1 && detected_lines[(j+1)%3].size() > 1)
                    for(size_t a=0; a < detected_lines[j].size(); a++)
                        for(size_t aa=a+1; aa < detected_lines[j].size(); aa++)
                            for(size_t b=0; b < detected_lines[(j+1)%3].size(); b++)
                                for(size_t bb=b+1; bb < detected_lines[(j+1)%3].size(); bb++)
                                {
                                    //两个LRF的四条直线
                                    CO CO_guess;
                                    CO_guess.resize(2);
                                    CO_guess[0].id_LRF = j;
                                    CO_guess[0].lines_co[0].center = Vector2d(1/detected_lines[j][a].second.coefs[0], (-detected_lines[j][a].second.coefs[2]-1)/detected_lines[j][a].second.coefs[1]);
                                    CO_guess[0].lines_co[0].cov_center = Matrix2d::Identity();
                                    CO_guess[0].lines_co[0].dir = Vector2d(-detected_lines[j][a].second.coefs[1], detected_lines[j][a].second.coefs[0]);
                                    CO_guess[0].lines_co[0].cov_dir = Matrix2d::Identity();

                                    CO_guess[0].lines_co[1].center = Vector2d(1/detected_lines[j][aa].second.coefs[0], (-detected_lines[j][aa].second.coefs[2]-1)/detected_lines[j][aa].second.coefs[1]);
                                    CO_guess[0].lines_co[1].cov_center = Matrix2d::Identity();
                                    CO_guess[0].lines_co[1].dir = Vector2d(-detected_lines[j][aa].second.coefs[1], detected_lines[j][aa].second.coefs[0]);
                                    CO_guess[0].lines_co[1].cov_dir = Matrix2d::Identity();

                                    CO_guess[1].id_LRF = (j+1)%3;
                                    CO_guess[1].lines_co[0].center = Vector2d(1/detected_lines[(j+1)%3][b].second.coefs[0], (-detected_lines[(j+1)%3][b].second.coefs[2]-1)/detected_lines[(j+1)%3][b].second.coefs[1]);
                                    CO_guess[1].lines_co[0].cov_center = Matrix2d::Identity();
                                    CO_guess[1].lines_co[0].dir = Vector2d(-detected_lines[(j+1)%3][b].second.coefs[1], detected_lines[(j+1)%3][b].second.coefs[0]);
                                    CO_guess[1].lines_co[0].cov_dir = Matrix2d::Identity();

                                    CO_guess[1].lines_co[1].center = Vector2d(1/detected_lines[(j+1)%3][bb].second.coefs[0], (-detected_lines[(j+1)%3][bb].second.coefs[2]-1)/detected_lines[(j+1)%3][bb].second.coefs[1]);
                                    CO_guess[1].lines_co[1].cov_center = Matrix2d::Identity();
                                    CO_guess[1].lines_co[1].dir = Vector2d(-detected_lines[(j+1)%3][bb].second.coefs[1], detected_lines[(j+1)%3][bb].second.coefs[0]);
                                    CO_guess[1].lines_co[1].cov_dir = Matrix2d::Identity();

                                    vCOs.push_back(CO_guess);

                                    // Generate reversed CO (a,aa,bb,b) because we don't really know the line-plane correspondences
                                    //为什么要制作相反的数据
                                    CO_guess[1].lines_co[0].center = Vector2d(1/detected_lines[(j+1)%3][bb].second.coefs[0], (-detected_lines[(j+1)%3][bb].second.coefs[2]-1)/detected_lines[(j+1)%3][bb].second.coefs[1]);
                                    CO_guess[1].lines_co[0].cov_center = Matrix2d::Identity();
                                    CO_guess[1].lines_co[0].dir = Vector2d(-detected_lines[(j+1)%3][bb].second.coefs[1], detected_lines[(j+1)%3][bb].second.coefs[0]);
                                    CO_guess[1].lines_co[0].cov_dir = Matrix2d::Identity();

                                    CO_guess[1].lines_co[1].center = Vector2d(1/detected_lines[(j+1)%3][b].second.coefs[0], (-detected_lines[(j+1)%3][b].second.coefs[2]-1)/detected_lines[(j+1)%3][b].second.coefs[1]);
                                    CO_guess[1].lines_co[1].cov_center = Matrix2d::Identity();
                                    CO_guess[1].lines_co[1].dir = Vector2d(-detected_lines[(j+1)%3][b].second.coefs[1], detected_lines[(j+1)%3][b].second.coefs[0]);
                                    CO_guess[1].lines_co[1].cov_dir = Matrix2d::Identity();

                                    vCOs.push_back(CO_guess);
                                }
}