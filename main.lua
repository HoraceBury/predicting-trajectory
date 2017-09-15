-- predicting trajectory

require("physics")
physics.start()
physics.setGravity(0,10)
physics.setDrawMode("hybrid")

sWidth, sHeight = display.contentWidth, display.contentHeight

require("mathlib")

gravity = {x=0,y=10}

prediction = display.newGroup()

faceDirectionOfTravel = true

line = nil

require("wall")

newWall{ name="left", }
newWall{ name="right", }
newWall{ name="top", }
newWall{ name="bottom", }

--b2Vec2 getTrajectoryPoint( b2Vec2& startingPosition, b2Vec2& startingVelocity, float n )
function getTrajectoryPoint( startingPosition, startingVelocity, n )
	-- velocity and gravity are given per second but we want time step values here
	local t = 1/display.fps -- float t = 1 / 60.0f; // seconds per time step (at 60fps)
	local stepVelocity = {x=t*startingVelocity.x,y=t*startingVelocity.y} -- b2Vec2 stepVelocity = t * startingVelocity; // m/s
	local stepGravity = { x=t*0, y=t*10 } -- b2Vec2 stepGravity = t * t * m_world->GetGravity(); // m/s/s
	return {
		x=startingPosition.x + n * stepVelocity.x + 0.5 * (n*n+n) * stepGravity.x,
		y=startingPosition.y + n * stepVelocity.y + 0.5 * (n*n+n) * stepGravity.y
	} -- startingPosition + n * stepVelocity + 0.5f * (n*n+n) * stepGravity;
end

function updatePrediction(e)
	prediction:removeSelf()
	prediction = display.newGroup()
	
	local startingVelocity = {x=e.x-e.xStart, y=e.y-e.yStart}
	
	-- draw line
	for i=0, 180 do -- for (int i = 0; i < 180; i++) { // three seconds at 60fps
		local s = {x=e.xStart,y=e.yStart}
		local trajectoryPosition = getTrajectoryPoint( s, startingVelocity, i ) -- b2Vec2 trajectoryPosition = getTrajectoryPoint( startingPosition, startingVelocity, i );
		display.newCircle( prediction, trajectoryPosition.x, trajectoryPosition.y, 5 )
	end
end

function fire(e)
	local cube = display.newRect(0,0,50,50)
	cube.x, cube.y = e.xStart, e.yStart
	physics.addBody(cube,{density=1})
	local vx, vy = e.x-e.xStart,e.y-e.yStart
	cube:setLinearVelocity(vx,vy)
	print('fire',e.x-e.xStart,e.y-e.yStart)
	
	if (faceDirectionOfTravel) then
		local start = system.getTimer()
		local function faceDirection()
			local vx, vy = cube:getLinearVelocity()
			local angle = angleOf({x=0,y=0},{x=vx,y=vy})
			cube.rotation = angle
			if (start+2000<system.getTimer()) then
				Runtime:removeEventListener("enterFrame",faceDirection)
			end
		end
		Runtime:addEventListener("enterFrame",faceDirection)
	end
end

function touch(e)
	if (e.phase == "began") then
		line = display.newLine(e.x,e.y,e.x,e.y)
		line.width = 5
	elseif (e.phase == "moved") then
		line:removeSelf()
		line = display.newLine(e.xStart,e.yStart,e.x,e.y)
		line.width = 5
		updatePrediction(e)
	else
		line:removeSelf()
		line = nil
		updatePrediction(e)
		fire(e)
	end
	return true
end
Runtime:addEventListener("touch",touch)

print( display.fps )
