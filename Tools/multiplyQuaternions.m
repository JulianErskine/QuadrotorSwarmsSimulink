function q = multiplyQuaternions(qA_,qB_)

sa = size(qA_);
sb = size(qB_);

if( sa(1) == 1 && sa(2) == 4)
  qA = reshape(qA_,[4,1]);
elseif(sa(1) ~= 4 || sa(2) ~= 1)
    error("qA must be a [4x1] vector")
else
  qA = qA_;
end

if( sb(1) == 1 && sb(2) == 4)
  qB = reshape(qB_,[4,1]);
elseif(sb(1) ~= 4 || sb(2) ~= 1)
    error("qA must be a [4x1] vector")
else
  qB = qB_;
end


q0 = qA(1);
q1 = qA(2);
q2 = qA(3);
q3 = qA(4);

Q = [q0, -q1, -q2, -q3;
  q1,  q0, -q3,  q2;
  q2,  q3,  q0, -q1;
  q3, -q2,  q1,  q0];

q = Q * qB;
end


