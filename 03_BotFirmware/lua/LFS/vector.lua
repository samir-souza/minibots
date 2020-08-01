require 'class'
require 'quaternion'

Vector = class()
Vector.X_AXIS = 0
Vector.Y_AXIS = 1
Vector.Z_AXIS = 2


function Vector:init(x,y,z)
    self.x = x or 0.0
    self.y = y or 0.0
    self.z = z or 0.0
end

function Vector:__tostring()
    return string.format("[%f,%f,%f]", self.x, self.y, self.z)
end

function Vector:copy()
    return Vector(self.x, self.y, self.z)
end

function Vector:__extract_params(v)
    if type(v) == 'number' then
        return v,v,v
    elseif type(v) == 'table' then
        return v.x, v.y, v.z
    else
        error("Vector:__extract_params: Invalid datatype " .. type(v))
    end
end

function Vector:__sub(v)
    local x,y,z = self:__extract_params(v)
    return Vector(self.x-x, self.y-y, self.z-z)
end

function Vector:__add(v)
    local x,y,z = self:__extract_params(v)
    return Vector(self.x+x, self.y+y, self.z+z)
end

function Vector:__mul(v)
    local x,y,z = self:__extract_params(v)
    return Vector(self.x*x, self.y*y, self.z*z)
end

function Vector:__div(v)
    local x,y,z = self:__extract_params(v)
    return Vector(self.x/x, self.y/y, self.z/z)
end

function Vector:get_magnitude() 
    return math.sqrt(self.x*self.x + self.y*self.y + self.z*self.z)
end

function Vector:rotate_axis(angle, axis)
    local cos, sin = math.cos(angle),math.sin(angle)
    if axis == 0 then -- X axis
        local y = self.y * cos - self.z * sin
        local z = self.y * sin + self.z * cos
        self.y, self.z = y,z
    elseif axis == 1 then -- Y axis
        local x = self.x * cos + self.z * sin
        local z = -self.x * sin + self.z * cos
        self.x,self.z = x,z
    else -- Z axis
        local x = self.x * cos - self.y * sin
        local y = self.x * sin + self.y * cos
        self.x,self.y = x,y
    end
end

-- compute the angle between the vectors
function Vector:get_angle(v)
    local cross = self:cross(v)
    return math.atan2(cross.x+cross.y+cross.z, self:dot(v))
end

function Vector:dot(v)
    local x,y,z = self:__extract_params(v)
    return self.x*x + self.y*y + self.z*z
end

function Vector:cross(v)
    local x,y,z = self:__extract_params(v)
    return Vector(
        self.y*z-self.z*y, 
        self.z*x-self.x*z, 
        self.x*y-self.y*x)
end

function Vector:normalize() 
    local m = self:get_magnitude()
    self.x = self.x / m
    self.y = self.y / m
    self.z = self.z / m
end

function Vector:get_normalized() 
    local r = Vector(self.x, self.y, self.z)
    r:normalize()
    return r
end

function Vector:rotate(q) 
    local p = Quaternion(0, self.x, self.y, self.z)

    -- quaternion multiplication: q * p, stored back in p
    p = q:get_product(p)

    -- quaternion multiplication: p * conj(q), stored back in p
    p = p:get_product(q:get_conjugate())

    -- p quaternion is now [0, self.x', self.y', self.z']
    self.x = p.x
    self.y = p.y
    self.z = p.z
end

function Vector:get_rotated(q) 
    r = self:copy()
    r:rotate(q)
    return r
end
