//
//  LInearAssignment.swift
//  demo
//
//  Created by Shreshth Saxena on 08/09/20.
//  Copyright © 2020 Apple. All rights reserved.
//

import Foundation
import Upsurge

public class HunSolver {
    private var matrix: Matrix<Double>
    private var matrixMax: Matrix<Double>?
    private var maximization: Bool
    private var maxdif: Int
    private var maximum: Double
    private var match: [Int]
    private var vis: [Bool]
    private var adjM: [[Int]]
    private var zeros: [(Int, Int)]

    // Init
    // Cost: O(n^2)
    public init?(matrix:[[Double]], maxim: Bool = false) {
        self.maximization = maxim
        self.match = [Int]()
        self.vis = [Bool]()
        self.adjM = [[Int]]()
        self.zeros = [(Int, Int)]()
        if matrix.count == 0 || matrix[0].count == 0 {
            return nil
        }
        let f = matrix.count
        let c = matrix[0].count
        self.maxdif = abs(f-c)
        let n = max(f,c)
        self.matrix = Matrix(rows: n, columns: n, repeatedValue: 0)
        self.maximum = 0
        if self.maximization {
            for i in 0..<f {
                for j in 0..<c {
                    if self.maximum < matrix[i][j] {
                        self.maximum = matrix[i][j]
                    }
                }
            }
            self.matrixMax = Matrix(rows: n, columns: n, repeatedValue: maximum)
        }
        for i in 0...n {
            if i < f {
                for j in 0...n {
                    if j < c {
                        if matrix[i][j] == Double.infinity || matrix[i][j].isNaN || matrix[i][j] < 0 {
                            return nil
                        }
                        if self.maximization {
                            self.matrix[i,j] = maximum - matrix[i][j]
                            self.matrixMax![i,j] = matrix[i][j]
                        } else {
                            self.matrix[i,j] = matrix[i][j]
                        }
                    }
                }
            }
        }
        if !self.maximization {
            self.matrixMax = nil
        }
    }

    // Solve the assignment problem
    // Cost: O(n^4)
    public func solve() -> (Double, [(Int, Int)]){
        var resultMatrix = self.matrix.copy()
        // 1: substract the minimum of each row
        for i in 0..<self.matrix.rows {
            let mi_minimo = resultMatrix.row(i).min()
            for j in 0..<self.matrix.columns {
                resultMatrix[i,j] -= mi_minimo!
            }
        }

        // 2: substract the minimum of each column
        let costesT = (self.matrix)′
        resultMatrix = (resultMatrix)′

        for i in 0..<costesT.rows {
            let mi_minimo = resultMatrix.row(i).min()
            for j in 0..<costesT.columns {
                resultMatrix[i,j] -= mi_minimo!
            }
        }
        resultMatrix = (resultMatrix)′

        // 3: Obtain the maximum matching in the reduced cost matrix
        var pos = [(Int,Int)] ()
        // In each loop, the cardinal of the matching increases at least in one, so this loop runs n times
        while (pos.count < self.matrix.rows){
            pos = obtainMatches(matrix: resultMatrix)
            if pos.count < self.matrix.rows {
                // 4: Substract the minimum to not crossed elements and add it to double crossed elements
                resultMatrix = adjustMatrix(resultMatrix, pos)
            }

        }

        // 5: Obtain the final cost
        var cost:Double = 0
        for (i,j) in pos {
            if self.maximization {
                cost += self.matrixMax![i,j]
            } else {
                cost += self.matrix[i,j]
            }
        }
        if self.maximization {
            cost -= self.maximum*Double(self.maxdif)
        }
        return (cost, pos)
    }


    // Create the lines through the matrix and add or substract the minimum
    // O(n^2)
    private func adjustMatrix(_ mat: Matrix<Double>, _ mark: [(Int,Int)]) -> Matrix<Double> {
        var rows = [[Int]](repeating: [Int](repeating: 0, count: mat.rows), count: mat.rows)
        var columns = [[Int]](repeating: [Int](repeating: 0, count: mat.rows), count: mat.rows)
        var markR = [Bool](repeating: false, count: mat.rows)
        var markC = [Bool](repeating: false, count: mat.rows)

        for elem in self.zeros {                                                        // O(n^2)
            rows[elem.0][elem.1] = 1
            columns[elem.1][elem.0] = 1
        }

        for elem in mark {                                                          // O(n)
            rows[elem.0][elem.1] = 2
            columns[elem.1][elem.0] = 2
        }

        for i in 0..<mat.rows {                                                         // O(n)
            if rows[i].max()! < 2 && !markR[i] {
                markR[i] = true
                if rows[i].max()! == 1 {
                    let idx = rows[i].indices.filter { rows[i][$0] == 1  }
                    for e in idx {
                        rows[i][e] = 3
                        columns[e][i] = 3
                    }
                }
            }
        }
        var auxf = markR
        var auxc = markC

        while true {                                                                    // O(n) * O(body) = O(n^2)
            for i in 0..<mat.rows {                                                             // O(n)
                if columns[i].max()! == 3 {
                    markC[i] = true
                    let idx = columns[i].indices.filter { columns[i][$0] == 2  }
                    for e in idx {
                        rows[e][i] = 0
                        columns[i][e] = 0
                    }
                }
            }

            for i in 0..<mat.rows {                                                             // O(n)
                if rows[i].max()! < 2 && !markR[i] {
                    markR[i] = true
                    if rows[i].max()! == 1 {
                        let idx = rows[i].indices.filter { rows[i][$0] == 1  }
                        for e in idx {
                            rows[i][e] = 3
                            columns[e][i] = 3
                        }
                    }
                }
            }
            if auxf == markR && auxc == markC {
                break
            } else {
                auxf = markR
                auxc = markC
            }
        }

        var minimum = Double.infinity
        for i in 0..<mat.rows {                                                         // O(n^2)
            if markR[i]{
                for j in 0..<mat.rows {
                    if !markC[j] && mat[i,j] < minimum {
                        minimum = mat[i,j]
                    }
                }
            }
        }
        for i in 0..<mat.rows {                                                         // O(n^2)
            if markR[i]{
                for j in 0..<mat.rows{
                    mat[i,j] -= minimum
                }
            }
            if !markC[i]{
                for j in 0..<mat.rows{
                    mat[j,i] -= minimum
                }
            }
        }
        for i in 0..<mat.rows {                                                         // O(n^2)
            for j in 0..<mat.rows {
                mat[i,j] += minimum
            }
        }

        return mat
    }

    private func augment(_ l: Int) -> Int {
        if self.vis[l] {
            return 0
        }
        self.vis[l] = true
        for i in 0..<self.adjM[l].count {
            let r = adjM[l][i]
            if self.match[r] == -1 || augment(self.match[r]) == 1 {
                self.match[r] = l
                return 1
            }
        }
        return 0
    }

    // Obtain maximum cost matching
    // Extracted from Competitive Programming 3 section 4.7.4
    // Coste: O(VE) = O(n^3)
    private func obtainMatches(matrix mat: Matrix<Double>) -> [(Int, Int)] {
        var MCBM = 0
        self.adjM = [[Int]]()
        self.zeros = [(Int,Int)]()
        for i in 0..<mat.rows {
            self.adjM.append([])
            for j in 0..<mat.rows {
                if mat[i,j] == 0 {
                    self.adjM[i].append(j + mat.rows)
                    self.zeros.append((i,j))
                }
            }
        }
        self.match = [Int](repeating: -1, count: mat.rows*2)
        for l in 0..<mat.rows {
            self.vis = [Bool](repeating: false, count: mat.rows)
            MCBM += augment(l)
        }
        var positions = [(Int,Int)]()
        for i in mat.rows..<mat.rows*2 {
            if self.match[i] != -1 {
                positions.append((self.match[i], i-mat.rows))
            }
        }
        return(positions)
    }
}
