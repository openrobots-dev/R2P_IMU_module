#ifndef _PID_HPP_
#define _PID_HPP_

template<typename T>
class PID {
private:
	T i_;
	T d_;
	T setpoint_;
	T kp_;
	T ki_;
	T kd_;
	T min_;
	T max_;

public:
	PID(T kp, T ki, T kd, T min, T max);
	void config(T kp, T ki, T kd);
	void set(T setpoint);
	T update(T measure, T dt);
};

template<typename T>
PID<T>::PID(T kp, T ki, T kd, T min, T max) :
		kp_(kp), ki_(ki), kd_(kd), min_(min), max_(max) {
	i_ = 0;
	d_ = 0;
	setpoint_ = 0;
}

template<typename T>
void PID<T>::config(T kp, T ki, T kd) {

	kp_ = kp;
	ki_ = ki;
	kd_ = kd;
}

template<typename T>
void PID<T>::set(T setpoint) {

	setpoint_ = setpoint;
}

template<typename T>
T PID<T>::update(T measure, T dt) {
	T error;
	T output;

	/* calculate error */
	error = setpoint_ - measure;

	/* proportional term */
	output = (kp_ * error);

	/* saturation filter */
	if (output > max_) {
		output = max_;
		/* conditional anti-windup: integrate only if error and setpoint have different sign */
		if ((error >= 0) && (setpoint_ <= 0)) {
			i_ += error * dt;
			output += (ki_ * i_);
		}
	} else if (output < min_) {
		output = min_;
		/* conditional anti-windup: integrate only if error and setpoint have different sign */
		if ((error >= 0) && (setpoint_ <= 0)) {
			i_ += error * dt;
			output += (ki_ * i_);
		}
	} else {
		i_ += error * dt;
		output += (ki_ * i_);
	}



	/* derivative term */
	output += (kd_ * (error - d_));
	d_ = error;

	return output;
}

#endif /* _PID_HPP_ */
