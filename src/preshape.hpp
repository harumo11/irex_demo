#pragma once

#include <cmath>
#include <deque>
#include <iomanip>
#include <iostream>

/**
 * @brief プリシェイプを行うためのクラス．
 */
class preshape {
public:
    preshape(const bool verbose = true, const double sample_freq = 100, const double delay_sec = 0.217);
    double step(const double u);

private:
    double freq; //サンプリング周波数
    double Tdelay; //遅れ時間
    double T; //制御周期
    int delay_step; //何ステップ前の値を参照するかの指標
    double u_half; //0.5倍された制御入力
    std::deque<double> u_half_history; //u_halfの履歴．サイズはdelay_stepにより指定される
    bool verbose; //trueなら内部の計算が表示される
    void print_u_half_history(); //u_half_historyをきれいに表示する関数
};

preshape::preshape(const bool verbose, const double sample_freq, const double delay_sec)
{
    this->freq = sample_freq;
    this->Tdelay = delay_sec;
    this->T = 1 / this->freq;
    this->delay_step = std::round(Tdelay / T);
    this->u_half = 0;
    this->u_half_history = std::deque<double>(this->delay_step, 0);
    this->verbose = verbose;

    if (verbose == true) {
        std::cout << "freq :\t\t" << this->freq << std::endl;
        std::cout << "Tdelay:\t\t" << this->Tdelay << std::endl;
        std::cout << "T:\t\t" << this->T << std::endl;
        std::cout << "delay_step:\t" << this->delay_step << std::endl;
        std::cout << "u_half:\t\t" << this->u_half << std::endl;
        std::cout << "u_half_delay" << std::endl;
        for (auto e : this->u_half_history) {
            std::cout << e << " ";
        }
        std::cout << std::endl;
    }
}

double preshape::step(const double u)
{
    //u_halfを計算
    this->u_half = 0.5 * u;
    if (this->verbose) {
        std::cout << "u_half:\t" << this->u_half << std::endl;
    }

    //u_half_delayの最後にu_halfを代入
    this->u_half_history.push_back(this->u_half);
    if (this->verbose) {
        std::cout << "u_half_history before" << std::endl;
        this->print_u_half_history();
    }

    //u_half_delayの要素を１つ前にずらす
    double u_half_delay = this->u_half_history.front();
    this->u_half_history.pop_front();
    if (this->verbose) {
        std::cout << "u_half_delay:\t" << u_half_delay << std::endl;
        std::cout << "u_half_history after" << std::endl;
        this->print_u_half_history();
    }
    //u = u_half + u_half_delay[0]
    return this->u_half + u_half_delay;
}

void preshape::print_u_half_history()
{
    for (auto e : this->u_half_history) {
        std::cout << std::setprecision(5) << e << " ";
    }
    std::cout << std::endl;
}
