require 'class'
Quaternion = class()

function Quaternion:init(w,x,y,z)
    self.w = w or 1.0
    self.x = x or 0.0
    self.y = y or 0.0
    self.z = z or 0.0
end

function Quaternion:get_product(q)
    -- Quaternion multiplication is defined by:
    --     (Q1 * Q2).w = (w1w2 - x1x2 - y1y2 - z1z2)
    --     (Q1 * Q2).x = (w1x2 + x1w2 + y1z2 - z1y2)
    --     (Q1 * Q2).y = (w1y2 - x1z2 + y1w2 + z1x2)
    --     (Q1 * Q2).z = (w1z2 + x1y2 - y1x2 + z1w2
    return Quaternion(
        self.w*q.w - self.x*q.x - self.y*q.y - self.z*q.z,  -- new w
        self.w*q.x + self.x*q.w + self.y*q.z - self.z*q.y,  -- new x
        self.w*q.y - self.x*q.z + self.y*q.w + self.z*q.x,  -- new y
        self.w*q.z + self.x*q.y - self.y*q.x + self.z*q.w)  -- new z
end

function Quaternion:get_conjugate()
    return Quaternion(self.w, -self.x, -self.y, -self.z)
end

function Quaternion:get_magnitude()
    return math.sqrt(self.w*self.w + self.x*self.x + self.y*self.y + self.z*self.z)
end

function Quaternion:normalize()
    local m = self:get_magnitude()
    self.w = self.w / m
    self.x = self.x / m
    self.y = self.y / m
    self.z = self.z / m
end

function Quaternion:get_normalized()
    local r = Quaternion(self.w, self.x, self.y, self.z)
    r:normalize()
    return r
end