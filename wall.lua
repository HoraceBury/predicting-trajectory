-- wall

--[[
	Description:
	
	Params:
	.name = left, right, top, bottom
	.parent
	.scene
	Returns:
	A wall
]]--
function newWall( params )
	local group = display.newGroup()
	--params.parent:insert( group )
	
	group.class = "wall"
	group.name = params.name
	group.params = params
	
	local width, height, x, y
	
	if (params.name == "left" or params.name == "right") then
		width, height = 5, sHeight
		y = sHeight/2
		if (params.name == "left") then
			x = 0
		else
			x = sWidth
		end
	elseif (params.name == "top" or params.name == "bottom") then
		width, height = sWidth, 5
		x = sWidth/2
		if (params.name == "top") then
			y = 0
		else
			y = sHeight
		end
	end
	
	group.rect = display.newRect( group, 0, 0, width, height )
	group.rect.x, group.rect.y = 0, 0
	group.x, group.y = x, y
	
	physics.addBody( group, "static" )
end
