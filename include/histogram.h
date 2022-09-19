//
// Created by bzdfzfer on 2022/8/26.
//

#ifndef HISTOGRAM_H
#define HISTOGRAM_H

#include <iostream>
#include <vector>
#include <cmath>
// #include "common.h"

using std::cout;
using std::endl;

class Histogram{
public:

    Histogram() {}

    Histogram(const Histogram& hist) {
        m_bin_num = hist.getBinNum();
        m_bin_start = hist.getBinStart();
        m_bin_stop = hist.getBinStop();
        m_bin_width = hist.getBinWidth();

        m_bin_means = hist.getBinMeans();
        m_bin_values = hist.getBinValues();
        m_bin_counts = hist.getBinCounts();
        m_bin_indexes = hist.getBinIndexes();

        m_valid_bin_idxs = hist.getValidBins();

        m_bin_valid_orders = hist.getBinValidOrders();
        m_valid_bin_labels = hist.getValidBinLabels();
    }

    Histogram(double min_value, double max_value, int bin_num);

    void process(const std::vector<double>& data_x_in, const std::vector<double>& data_y_in);

    void process(const std::vector<double>& data_x_in);

    void set(double min_value, double max_value, int bin_num) {
        
        m_bin_num = bin_num;
        m_bin_start = min_value;
        m_bin_stop = max_value;
        m_bin_width = (m_bin_stop - m_bin_start)/m_bin_num;
    }

    void setBinCenterWidthNum(double min_bin_center, double bin_width, int bin_num) {
        m_bin_num = bin_num;
        m_bin_width = bin_width;
        m_bin_start = min_bin_center - m_bin_width/2;
        m_bin_stop = min_bin_center + (m_bin_num-1)*m_bin_width + m_bin_width/2;
    }

    void setBinMaxMinWidth(double max_value, double min_value, double bin_width) {
        m_bin_start = min_value;
        m_bin_stop = max_value;
        m_bin_width = bin_width;

        m_bin_num = (m_bin_stop - m_bin_start) / m_bin_width;
    }

    void setBinNum(int num) { m_bin_num = num; }
    void setBinStart(double start) { m_bin_start = start; }
    void setBinStop(double stop) { m_bin_stop = stop; }
    void steBinWidth(double width) { m_bin_width = width; }

    void setAbsorbBinNum(int absorb_bin) { m_absorb_bin_num = absorb_bin; }


    void setBinMeans(const std::vector<double>& bin_means) { m_bin_means = bin_means; }
    void setBinValues(const std::vector<double>& bin_values) { m_bin_values = bin_values; }
    void setBinCounts(const std::vector<int>& bin_counts) { m_bin_counts = bin_counts; }
    void setBinIndexes(const std::vector<std::vector<int>>& bin_indexes) { m_bin_indexes = bin_indexes; }

    void setBinMeanAt(double mean, int idx) { m_bin_means[idx] = mean; }
    void setBinValueAt(double value, int idx) { m_bin_values[idx] = value; }
    void setBinCountAt(int count, int idx) { m_bin_counts[idx] = count; }
    void setBinIndexesAt(const std::vector<int>& indexes, int idx) { m_bin_indexes[idx] = indexes; }

    void pushValidBinIdx(int valid_bin_idx) { m_valid_bin_idxs.push_back(valid_bin_idx); }

    int getBinNum() const { return m_bin_num; }
    double getBinStart() const { return m_bin_start; }
    double getBinStop() const { return m_bin_stop; }
    double getBinWidth() const { return m_bin_width; }

    std::vector<double> getBinMeans() const { return m_bin_means; }
    std::vector<double> getBinValues() const { return m_bin_values; }
    std::vector<int> getBinCounts() const { return m_bin_counts; }
    std::vector<std::vector<int>> getBinIndexes() const { return m_bin_indexes; }
    std::vector<int> getValidBins() const { return m_valid_bin_idxs; }

    int getValidBinNum() const { return m_valid_bin_idxs.size(); }
    int getValidBinIdxAt(int idx) const { return m_valid_bin_idxs[idx]; }

    double getBinMeanAt(int bin_idx) const { return m_bin_means[bin_idx]; }
    double getBinValueAt(int bin_idx) const { return m_bin_values[bin_idx]; }
    int getBinCountAt(int bin_idx) const { return m_bin_counts[bin_idx]; }
    std::vector<int> getBinIndexesAt(int bin_idx) const { return m_bin_indexes[bin_idx]; }

    std::vector<int> getValidBinLabels() const { return m_valid_bin_labels; }
    std::vector<int> getBinValidOrders() const { return m_bin_valid_orders; }
    int getValidBinLabelAt(int valid_bin_idx) const { return m_valid_bin_labels[valid_bin_idx]; }
    int getBinValidOrderAt(int bin_idx) const { return m_bin_valid_orders[bin_idx]; }
    int getBinLabelAt(int bin_idx) const { return getValidBinLabelAt(getBinValidOrderAt(bin_idx)); }
    int getBinClusterNum() { return m_clusters_num; }

    std::vector<double> getValidBinMeans() const {
        std::vector<double> valid_bin_means;
        for(int i=0; i < m_valid_bin_idxs.size(); i++) {
            int valid_idx = m_valid_bin_idxs[i];
            valid_bin_means.push_back(m_bin_means[valid_idx]);
        }
        return valid_bin_means;
    }

    std::vector<double> getValidBinValues() {
        std::vector<double> valid_bin_values;
        for(int i=0; i < m_valid_bin_idxs.size(); i++) {
            int valid_idx = m_valid_bin_idxs[i];
            valid_bin_values.push_back(m_bin_values[valid_idx]);
        }
        return valid_bin_values;
    }

    std::vector<std::vector<int>> getValidBinIndexes() {
        std::vector<std::vector<int>> valid_bin_idxs;
        for(int i=0; i < m_valid_bin_idxs.size(); i++) {
            int valid_idx = m_valid_bin_idxs[i];
            valid_bin_idxs.push_back(m_bin_indexes[valid_idx]);
        }
        return valid_bin_idxs;
    }

    // input: valid bin idxs.
    // output: 
    //         valid bin sets.

    std::vector<int> clusterAdjacentValidBins(int& label) {

        label = 0;
        if(m_valid_bin_num==0)
            return std::vector<int>();

        std::vector<int> clusters_label(m_valid_bin_num, -1);
        if(m_valid_bin_num==1)
        {
            clusters_label[0] = 0;
            label ++;
            return clusters_label;
        }
        
        for(int i=0; i < m_valid_bin_num-1;i++) {
            int bin_idx = m_valid_bin_idxs[i];
            int j = i+1;
            int n_bin_idx = m_valid_bin_idxs[j];
            int diff_idx = n_bin_idx - bin_idx;
            while(diff_idx <= m_absorb_bin_num && j < m_valid_bin_num-1) {
                diff_idx = m_valid_bin_idxs[j+1] - m_valid_bin_idxs[j];
                j++;
            }

            j--;
            for(int k=i; k<=j; k++) {
                clusters_label[k] = label;
            }
            i = j;
            label ++;
        }

        if(m_valid_bin_idxs[m_valid_bin_num-1] - m_valid_bin_idxs[m_valid_bin_num-2] <= m_absorb_bin_num)
        {
            label --;   
        } 
        clusters_label[m_valid_bin_num-1] = label;      


        // handle head and tails 
        // labels.
        int head_tail_gap = m_bin_num - m_valid_bin_idxs.back() + m_valid_bin_idxs.front();
        // if(m_valid_bin_idxs.front() == 0 && m_valid_bin_idxs.back()==m_bin_num-1) {
        if( head_tail_gap > 0 && head_tail_gap <= m_absorb_bin_num ) {
            for(int i=m_valid_bin_num-1; i >= 0; i--) {
                if(clusters_label[i]==label) {
                    clusters_label[i] = 0;
                } else {
                    break;
                }
            }
            label --;
        }

        return clusters_label;
    }

    static Histogram* createShiftedHistogram(const Histogram& hist, int bin_shift) {
        Histogram* hist_ptr = new Histogram(hist);

        hist_ptr->clear();
        std::vector<int> origin_valid_bins = hist.getValidBins();
        for(int i=0; i < origin_valid_bins.size(); i++) {
            int origin_bin_idx = origin_valid_bins[i];

            int new_bin_idx = origin_bin_idx + bin_shift;
            if(new_bin_idx >= hist_ptr->getBinNum())
                new_bin_idx -= hist_ptr->getBinNum();
            else if(new_bin_idx < 0)
                new_bin_idx += hist_ptr->getBinNum();

            auto bin_mean = hist.getBinMeanAt(origin_bin_idx);
            auto bin_value = hist.getBinValueAt(origin_bin_idx);
            auto bin_count = hist.getBinCountAt(origin_bin_idx);
            auto bin_indexes = hist.getBinIndexesAt(origin_bin_idx);

            hist_ptr->setBinMeanAt(bin_mean, new_bin_idx);
            hist_ptr->setBinValueAt(bin_value, new_bin_idx);
            hist_ptr->setBinCountAt(bin_count, new_bin_idx);
            hist_ptr->setBinIndexesAt(bin_indexes, new_bin_idx);

            hist_ptr->pushValidBinIdx(new_bin_idx);

        }
        return hist_ptr;
    }

    void clear();
protected:



private:
    int m_bin_num;

    double m_bin_start;
    double m_bin_stop;
    double m_bin_width;

    std::vector<double> m_bin_means;
    std::vector<double> m_bin_values;
    std::vector<int> m_bin_counts;
    std::vector<int> m_bin_valid_orders;

    std::vector<std::vector<int>> m_bin_indexes;

    std::vector<int> m_valid_bin_idxs;
    int m_valid_bin_num;

    int m_clusters_num;
    std::vector<int> m_valid_bin_labels;

    int m_absorb_bin_num = 1;   
};



#endif
