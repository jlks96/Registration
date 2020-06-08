#pragma once
#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

struct node {
    double node_val[3];
    int vertex_index;
    struct node *left, *right;
};

inline double dist(struct node *a, struct node *b) {
    double distance = 0;
    for (int i = 0; i < 3; i++) {
        distance += (a->node_val[i] - b->node_val[i]) * (a->node_val[i] - b->node_val[i]);
    }
    return distance;
}

inline void swap(struct node *a, struct node *b) {
    double temp[3];
    memcpy(temp, a->node_val, sizeof(temp));
    memcpy(a->node_val, b->node_val, sizeof(temp));
    memcpy(b->node_val, temp, sizeof(temp));
}

// partition like median finding
struct node* findMedian(struct node *start, struct node *end, int idx) {
    if (end <= start) return NULL;
    if (end == start + 1) {
        return start;
    }

    struct node *a, *b, *mid = start + (end - start) / 2;
    double pivot;
    while (1) {
        pivot = mid->node_val[idx];
        swap(mid, end - 1);
        for (b = a = start; a < end; a++) {
            if (a->node_val[idx] < pivot) {
                if (a != b)
                    swap(a, b);
                b++;
            }
        }
        swap(b, end - 1);

        if (b->node_val[idx] == mid->node_val[idx])
            return mid;

        if (b > mid) end = b;
        else start = b;
    }
}

// recursively build tree
struct node* buildTree(struct node *t, int len, int i) {
    struct node *median;
    if (!len) return 0;

    if ((median = findMedian(t, t + len, i))) {
        i = (i + 1) % 3;
        median->left = buildTree(t, median - t, i);
        median->right = buildTree(median + 1, t + len - (median + 1), i);
    }
    return median;
}

/*
// go down tree to find closest point
void findNearest(struct node *root, struct node *input_node, int i, struct node **best, double best_dist) {
    double d, dx, dx2;

    if (!root) return;
    d = dist(root, input_node, 3);
    dx = root->node_val[i] - input_node->node_val[i];
    dx2 = dx * dx;

    // if current distance is better, update best_dist node
    if (!*best || d < best_dist) {
        best_dist = d;
        *best = root;
    }

    // exact match
    if (!best_dist) return;

    // go to next dimension
    i++;
    if (i >= 3) i = 0;

    findNearest(dx > 0 ? root->left : root->right, input_node, i, best, best_dist);
    if (dx2 >= best_dist) return;
    findNearest(dx > 0 ? root->right : root->left, input_node, i, best, best_dist);
}
*/
void findNearest(struct node *root, struct node *input_node, int i, struct node **best, double *best_dist) {
    double d, dx, dx2;

    if (!root) return;
    d = dist(root, input_node);
    dx = root->node_val[i] - input_node->node_val[i];
    dx2 = dx * dx;

    // if current distance is better, update best_dist node
    if (!*best || d < *best_dist) {
        *best_dist = d;
        *best = root;
    }

    // exact match
    if (!best_dist) return;

    // go to next dimension
    //i++;
    //if (++i >= 3) i = 0;
    if (++i >= 3) i = 0;

    findNearest(dx > 0 ? root->left : root->right, input_node, i, best, best_dist);
    if (dx2 >= *best_dist) return;
    findNearest(dx > 0 ? root->right : root->left, input_node, i, best, best_dist);
}