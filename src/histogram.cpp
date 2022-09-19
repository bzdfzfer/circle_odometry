//
// Created by bzdfzfer on 2022/8/26.
//

#include "histogram.h"

Histogram::Histogram(double min_value, double max_value, int bin_num) :
    m_bin_num(bin_num),
    m_bin_start(min_value),
    m_bin_stop(max_value)
{
    m_bin_width = (m_bin_stop - m_bin_start)/m_bin_num;

}

void Histogram::process(const std::vector<double>& data_x_in) {

    // clear data first.
    clear();

    // assign bin values.
    for( int i=0; i < data_x_in.size(); i++) {
        double vx;
        vx = data_x_in[i];      
        int bin_idx = std::floor( (vx - m_bin_start)/m_bin_width );

        // wrap out of range bins.
        if(bin_idx < 0)
            bin_idx = 0;
        else if(bin_idx>= m_bin_num)
            bin_idx = m_bin_num -1;     

        m_bin_means[bin_idx] += vx;
        m_bin_values[bin_idx] ++;
        m_bin_counts[bin_idx] ++;

        m_bin_indexes[bin_idx].push_back(i);
    }

    m_valid_bin_num = 0;
    for(int i=0; i < m_bin_num; i++) {
        if(m_bin_counts[i]==0)
            continue;

        m_bin_means[i] /= m_bin_counts[i];
        m_valid_bin_idxs.push_back(i);
        m_bin_valid_orders[i] = m_valid_bin_num++;
    }

    m_valid_bin_labels = clusterAdjacentValidBins(m_clusters_num);
}

void Histogram::process(const std::vector<double>& data_x_in, const std::vector<double>& data_y_in) {

    // clear data first.
    clear();

    // assign hist bins.
    for(int i=0; i < data_x_in.size(); i++) {
        double vx, vy;
        vx = data_x_in[i];
        vy = data_y_in[i];
        int bin_idx = std::floor( (vx - m_bin_start)/m_bin_width );

        // wrap out of range bins.
        if(bin_idx < 0)
            bin_idx = 0;
        else if(bin_idx>= m_bin_num)
            bin_idx = m_bin_num -1;

        m_bin_means[bin_idx] += vx;
        m_bin_values[bin_idx] += vy;
        m_bin_counts[bin_idx] ++;

        m_bin_indexes[bin_idx].push_back(i);
    }

    m_valid_bin_num = 0;

    for(int i=0; i < m_bin_num; i++) {
        if(m_bin_counts[i]==0)
            continue;

        m_bin_means[i] /= m_bin_counts[i];
        m_bin_values[i] /= m_bin_counts[i];
        m_valid_bin_idxs.push_back(i);
        m_bin_valid_orders[i] = m_valid_bin_num++;      
    }

    m_valid_bin_labels = clusterAdjacentValidBins(m_clusters_num);

}

void Histogram::clear() {
    m_bin_means.resize(m_bin_num);
    m_bin_values.resize(m_bin_num);
    m_bin_counts.resize(m_bin_num);
    m_bin_indexes.resize(m_bin_num);
    m_bin_valid_orders.resize(m_bin_num);


    for(int i=0; i < m_bin_num; i++) {
        m_bin_means[i] = 0;
        m_bin_values[i] = 0;
        m_bin_counts[i] = 0;
        m_bin_indexes[i].clear();
        m_bin_valid_orders[i] = -1;
    }   

    m_valid_bin_idxs.clear();
}