#!/usr/bin/env python
import sys, time, os
import numpy as np
from scipy import linalg


class ProMP:

    def __init__(self, ndemos, obs_dofs, robot_dofs, training_address):

        self.ndemos = ndemos
        self.obs_dofs = obs_dofs
        self.robot_dofs = robot_dofs

        self.stdev = 0.005
        self.dt = 0.01

        self.count = 0
        self.start = 1

        self.p_data, self.q_data = self.loadData(self.dt, training_address)
        self.promp = self.pmpRegression(self.q_data)

        self.param = {"nTraj": self.p_data["size"], "nTotalJoints": self.promp["nJoints"], "observedJointPos": np.array(range(obs_dofs)), "observedJointVel": np.array([])}

        print "Initialized"

    def predictTraining(self, demo=0):
        '''
        
        '''
        self.phase = 99
        obs = self.q_data[demo][self.phase, :self.obs_dofs]
        print obs
        return self.predictTest(obs)

    def predictTest(self, obs, phase=99):
        """
        ProMP Estimation
        :param obs:
        :param phase:
        :return:
        """

        self.obs_pose = obs
        self.phase = phase
        obs = self.Observation(self.stdev, self.param, self.p_data, self.obs_pose)
        self.kf = self.kfLoop(self.promp, self.param, obs)

        return self.kf["q_mean"]

    def loadData(self, dt, training_address):
        robot_data, obs_data = [], []
        for i in range(self.ndemos):
            robot = np.loadtxt(open(training_address + "/robot_demo_"+ str(i+1) +".csv", "rb"), delimiter=",")
            robot_data.append(robot)
            obs = np.loadtxt(open(training_address + "/object_demo_"+ str(i+1) +".csv", "rb"), delimiter=",")
            obs_data.append(obs)

        robot, robot_mean = self.addVelocity(robot_data)
        obs, obs_mean = self.addVelocity(obs_data)

        demo_data = []
        for i in range(len(robot)):
            demo_data.append(np.concatenate((obs[i], robot[i]),axis=1))
        demo_mean = np.concatenate((obs_mean,robot_mean), axis=1)

        demo = {"q":[], "qdot":[], "q_mean":[], "q_cov":[], "q_var":[], "qdot_mean":[], "qdot_cov":[], "qdot_var":[]}
        for i in range(demo_data[0].shape[1]/2):
            q = q_dot = []
            for j in range(len(demo_data)):
                q.append(demo_data[j][:,2*i].T)
                q_dot.append(demo_data[j][:,(2*i)+1].T)    #TODO: Velocity not recalculated like in MATLAB. VERIFY!!???
            demo["q"].append(np.matrix(q))
            demo["qdot"].append(np.matrix(q_dot))

        for i in range(demo_data[0].shape[1]/2):
            demo["q_mean"].append(np.mean(demo["q"][i],axis=0))
            demo["q_cov"].append(np.cov(demo["q"][i].T))
            demo["q_var"].append(np.var(demo["q"][i],axis=0,ddof=1).T)
            demo["qdot_mean"].append(np.mean(demo["qdot"][i],axis=0))
            demo["qdot_cov"].append(np.cov(demo["qdot"][i].T))
            demo["qdot_var"].append(np.var(demo["qdot"][i],axis=0,ddof=1).T)

        demo["size"] = demo_data[0].shape[0]

        demo_q = []
        for i in range(len(demo_data)):
            q = []
            for j in range(demo_data[0].shape[1]/2):
                q.append(demo_data[i][:,2*j])
            demo_q.append(np.matrix(q).T)

        print "Data Loaded"

        return demo, demo_q

    def addVelocity(self, data):
        for k in range(len(data)):
            d = data[k]

            vel = np.zeros((d.shape[0],2*d.shape[1]))
            for i in range(d.shape[1]):
                vel[:,2*i] = d[:,i]
                for j in range(1,d.shape[0]):
                    vel[j,(2*i)+1] = (d[j,i] - d[j-1,i])/self.dt
            vel_d = np.zeros((100,vel.shape[1]))
            for i in range(vel.shape[1]):
                xo = np.linspace(0,99,100)
                xp = np.linspace(0,99,d.shape[0])
                vel_d[:,i] = np.interp(xo,xp,vel[:,i])
            data[k] = vel_d

        mean = sum(data)/len(data)

        return data, mean

    def pmpRegression(self, data, nBasis=30):
        nJoints = data[0].shape[1]
        nDemo = len(data)
        nTraj = data[0].shape[0]

        mu_location = np.linspace(0, 1, nBasis)
        phase = self.Phase(self.dt)

        weight = {"nBasis":nBasis, "nJoints":nJoints, "nTraj":nTraj, "nDemo":nDemo}
        weight["my_linRegRidgeFactor"] = 1e-08 * np.identity(nBasis)

        sigma = 0.05 * np.ones((1, nBasis))

        basis = self.generateGaussianBasis(phase, mu_location, sigma)

        weight = self.leastSquareOnWeights(weight, basis["Gn"], data)

        pmp = {"phase":phase, "w":weight, "basis":basis, "nBasis":nBasis, "nJoints":nJoints, "nDemo":nDemo, "nTraj":nTraj}

        return pmp

    def Phase(self,t):
        phase = {"dt":t}
        phase["z"] = np.linspace(t, 1, 1/t)
        zd = np.diff(phase["z"])/t
        phase["zd"] = np.append(zd,zd[-1])
        zdd = np.diff(phase["zd"])/t
        phase["zdd"] = np.append(zdd,zdd[-1])

        return phase

    # def Weight(nBasis,nJoints,nTraj,nDemo):
    #     weight = {"nBasis":nBasis, "nJoints":nJoints, "nTraj":nTraj, "nDemo":nDemo}
    #     weight["my_linRegRidgeFactor"] = 1e-08 * np.ones((nBasis,nBasis))


    def generateGaussianBasis(self,phase,mu,sigma):
        basisCenter = mu
        z = phase["z"]
        zd = phase["zd"]
        zdd = phase["zdd"]

        z_minus_center = np.matrix(z).T - np.matrix(basisCenter)

        at = np.multiply(z_minus_center, (1.0/sigma))

        Basis = {}

        basis = np.multiply(np.exp(-0.5*np.power(at,2)), 1./sigma/np.sqrt(2*np.pi))
        basis_sum = np.sum(basis, axis=1)
        basis_n = np.multiply(basis, 1.0/basis_sum)

        z_minus_center_sigma = np.multiply(-z_minus_center, 1.0/np.power(sigma,2))
        basisD = np.multiply(z_minus_center_sigma, basis)

        # normalizing basisD
        basisD_sum = np.sum(basisD, axis=1)
        basisD_n_a = np.multiply(basisD, basis_sum)
        basisD_n_b = np.multiply(basis, basisD_sum)
        basisD_n = np.multiply(basisD_n_a - basisD_n_b, 1.0/np.power(basis_sum,2))

        # second derivative of the basis
        tmp = np.multiply(basis, -1.0/np.power(sigma,2))
        basisDD = tmp + np.multiply(z_minus_center_sigma, basisD)
        basisDD_sum = np.sum(basisDD, axis=1)

        # normalizing basisDD
        basisDD_n_a = np.multiply(basisDD, np.power(basis_sum,2))
        basisDD_n_b1 = np.multiply(basisD, basis_sum)
        basisDD_n_b = np.multiply(basisDD_n_b1, basisD_sum)
        basisDD_n_c1 = 2 * np.power(basisD_sum,2) - np.multiply(basis_sum, basisDD_sum)
        basisDD_n_c = np.multiply(basis, basisDD_n_c1)
        basisDD_n_d = basisDD_n_a - 2 * basisDD_n_b + basisDD_n_c
        basisDD_n = np.multiply(basisDD_n_d, 1.0/np.power(basis_sum,3))

        basisDD_n = np.multiply(basisDD_n, np.matrix(np.power(zd,2)).T) + np.multiply(basisD_n, np.matrix(zdd).T)
        basisD_n = np.multiply(basisD_n, np.matrix(zd).T)

        Basis["Gn"] = basis_n
        Basis["Gndot"] = basisD_n
        Basis["Gnddot"] = basisDD_n

        return Basis

    def leastSquareOnWeights(self,weight,Gn,data):
        weight["demo_q"] = data
        nDemo = weight["nDemo"]
        nJoints = weight["nJoints"]
        nBasis = weight["nBasis"]
        my_linRegRidgeFactor = weight["my_linRegRidgeFactor"]

        MPPI = np.linalg.solve(Gn.T*Gn + my_linRegRidgeFactor, Gn.T)

        w, ind = [], []
        for i in range(nJoints):
            w_j = np.empty((0,nBasis), float)
            for j in range(nDemo):
                w_ = MPPI*data[j][:, i]
                w_j = np.append(w_j, w_.T, axis=0)
            w.append(w_j)
            ind.append(np.matrix(range(i*nBasis, (i+1)*nBasis)))

        weight["index"] = ind
        weight["w_full"] = np.empty((nDemo,0), float)
        for i in range(nJoints):
            weight["w_full"] = np.append(weight["w_full"], w[i], axis=1)

        weight["cov_full"] = np.cov(weight["w_full"].T)
        weight["mean_full"] = np.mean(weight["w_full"], axis=0).T

        return weight

    def Observation(self, stdev, param, p_data, obs_data):
        obs = {"joint": param["observedJointPos"], "jointvel": param["observedJointVel"], "stdev": stdev}
        obs["q"] = np.zeros((param["nTotalJoints"], param["nTraj"]))
        obs["qdot"] = np.zeros((param["nTotalJoints"], param["nTraj"]))
        obs["index"] = [99]

        for i in obs["joint"]:
            obs["q"][i, obs["index"]] = obs_data[0, i]

        return obs

    def kfLoop(self, promp, param, obs):
        sigma_obs = obs["stdev"]
        P0 = promp["w"]["cov_full"]
        x0 = promp["w"]["mean_full"]
        R_obs = (sigma_obs**2)*np.identity(2*param["nTotalJoints"])

        for k in obs["index"]:
            H0 = self.observationMatrix(k, promp, obs["joint"], obs["jointvel"])

            z0 = np.empty((0, 0), float)
            for i in range(promp["nJoints"]):
                z0 = np.append(z0, obs["q"][i, k])
                z0 = np.append(z0, obs["qdot"][i, k])
            z0 = np.matrix(z0).T

            x0, P0 = self.kfRecursion(x0, P0, H0, z0, R_obs)

            jointKF = self.perJointPromp(x0, P0, promp)

        return jointKF



    def kfRecursion(self, x_old, P_old, H, z, R_obs):
        H, P_old = np.matrix(H), np.matrix(P_old)
        tmp = np.matmul(H,np.matmul(P_old, H.T)) + R_obs
        K = np.matmul(np.matmul(P_old, H.T), np.linalg.inv(tmp))

        P_new = P_old - (K*H*P_old)

        x_new = x_old + K*(z - (H*x_old))

        return x_new, P_new

    def observationMatrix(self, k, p, observedJointPos, observedJointVel):
        nJoints = p["nJoints"]
        nTraj = p["nTraj"]
        Gn = p["basis"]["Gn"]
        Gn_d = p["basis"]["Gndot"]
        normalizedTime = k

        Hq_measured = Gn[normalizedTime, :]
        Hqdot_measured = Gn_d[normalizedTime, :]
        Hq_unmeasured = np.zeros((1, p["nBasis"]))
        Hqdot_unmeasured = np.zeros((1, p["nBasis"]))

        H = []
        for i in range(nJoints):
            if not observedJointPos.all:
                H_temp = Hq_unmeasured
            else:
                if np.sum(i == observedJointPos) == 0:
                    H_temp = Hq_unmeasured
                else:
                    H_temp = Hq_measured

            if not observedJointVel: #when joint vel not observed
                H_temp = np.append(H_temp, Hqdot_unmeasured, axis = 0)
            else:
                if np.sum(j==observedJointVel)==0:
                    H_temp = np.append(H_temp, Hqdot_unmeasured, axis = 0)
                else:
                    H_temp = np.append(H_temp, Hqdot_measured, axis = 0)

            H.append(H_temp)
        H = linalg.block_diag(*H)

        return H

    def perJointPromp(self, xFull, Pfull,pmp):
        nBasis = pmp["nBasis"]
        nTraj = pmp["nTraj"]

        kf = {"w_mean": [], "w_sigma": [], "w_sigma_ii": [], "q_mean": [], "q_sigma_ii": [], "qdot_mean": [], "qdot_sigma_ii": []}
        for i in range(pmp["nJoints"]):
            ind = np.array(range(i*nBasis,(i+1)*nBasis))
            kf["w_mean"].append(xFull[ind])
            kf["w_sigma"].append(Pfull[ind,:][:,ind])
            kf["w_sigma_ii"].append(np.diag(kf["w_sigma"][i]))

            q_mean, q_sigma_ii, qdot_mean, qdot_sigma_ii = self.thetaToTraj(kf["w_mean"][i], kf["w_sigma"][i], pmp["basis"], pmp["phase"]["dt"], nTraj)

            kf["q_mean"].append(q_mean)
            kf["q_sigma_ii"].append(q_sigma_ii)
            kf["qdot_mean"].append(qdot_mean)
            kf["qdot_sigma_ii"].append(qdot_sigma_ii)

        return kf

    def thetaToTraj(self, w_mean, P_w, basis, phase_dt, nTraj):
        x_mean = []
        x_sigma_ii = []
        xdot_mean = []
        xdot_sigma_ii = []

        for i in range(nTraj-1):
            timePoint = i*phase_dt
            mu_x,_,sigma_t = self.getDistribtionsAtTimeT1(w_mean,P_w,basis,phase_dt,timePoint)
            x_mean = np.append(x_mean, mu_x[0])
            xdot_mean = np.append(xdot_mean, mu_x[1])
            x_sigma_ii = np.append(x_sigma_ii, sigma_t[0,0])                   #TODO: check indexing
            xdot_sigma_ii = np.append(x_sigma_ii, sigma_t[1,1])

        return x_mean, x_sigma_ii, xdot_mean, xdot_sigma_ii

    def getDistribtionsAtTimeT1(self, w_mu, w_cov, basis, dt, timePoint):
        timePointIndex = int(round(timePoint/dt))

        Psi_t = basis["Gn"][timePointIndex,:]
        Psi_td = basis["Gndot"][timePointIndex,:]
        Psi_tdd = basis["Gnddot"][timePointIndex,:]

        Psi_t1 = basis["Gn"][timePointIndex+1,:]
        Psi_t1d = basis["Gndot"][timePointIndex+1,:]

        Phi_t = np.append(Psi_t.T, Psi_td.T, axis=1)
        Phi_t1 = np.append(Psi_t1.T, Psi_t1d.T, axis=1)
        Phi_td = np.append(Psi_td.T, Psi_tdd.T, axis=1)

        mu_x = Phi_t.T * w_mu
        mu_xd = Phi_td.T * w_mu
        # print w_cov.shape
        sigma_t = Phi_t.T * w_cov * Phi_t
        sigma_t1 = Phi_t1.T * w_cov * Phi_t1
        sigma_t_t1 = Phi_t.T * w_cov * Phi_t1
        sigma_td_half = Phi_td.T * w_cov * Phi_t

        return mu_x, mu_xd, sigma_t #, sigma_t1, sigma_t_t1, sigma_td_half

def main(args):
    pmp = ProMP()

if __name__ == '__main__':
    main(sys.argv)