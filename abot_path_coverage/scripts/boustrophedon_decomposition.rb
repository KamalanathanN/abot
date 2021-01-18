#!/usr/bin/ruby

require 'tempfile'
require 'json'
#require 'pry'
require 'logger'


# Ruby implementation of Coverage Path Planning: The Boustrophedon Decomposition
# https://www.ri.cmu.edu/publications/coverage-path-planning-the-boustrophedon-decomposition/
module BoustrophedonDecomposition
	@logger = Logger.new(STDERR)

	class Cell
		attr_reader :neighbours

		def initialize(points_ceil = [], points_floor = [])
			@points_ceil = points_ceil
			@points_floor = points_floor
			@neighbours = []
		end

		def add_floor_point(point)
			@points_floor.push(point)
		end

		def add_ceil_point(point)
			@points_ceil.push(point)
		end

		def rm_ceil_point()
			return @points_ceil.pop()
		end

		def get_last_floor_point()
			return @points_floor[-1]
		end

		# returns all points in counter clockwise order
		def points()
			filter()
			return @points_floor + @points_ceil.reverse
		end

		def to_s()
			return points.to_s
		end

		def filter()
			@points_floor = filter_points(@points_floor)
			@points_ceil = filter_points(@points_ceil)
		end

		# keep only points with different slope, aka remove all points that do not carry any additional information
		def filter_points(points)
			point_pre = nil
			angle_pre = nil
			points_filtered = []
			points.each do |point|
				if point_pre == nil
					# always keep first
					points_filtered.push(point)
				else
					angle_cur = Math.atan2(point[1] - point_pre[1], point[0]-point_pre[0])
					if angle_pre != angle_cur
						points_filtered.push(point)
					else
						# same slope as previous, keep this, remove previous
						points_filtered.pop()
						points_filtered.push(point)
					end
					angle_pre = angle_cur
				end
				point_pre = point
			end
			return points_filtered
		end

		# add neighbour cell
		def add_neighbour_end(cell)
			if not @neighbours.include? cell
				@neighbours.push cell 
				cell.add_neighbour_front(self)
			end
		end

		def add_neighbour_front(cell)
			if not @neighbours.include? cell
				@neighbours.unshift cell 
				cell.add_neighbour_end(self)
			end
		end

		# from geokit/polygon.rb
		def contains?(point)
			_points = points + points[0]
			last_point = _points[-1]
			oddNodes = false
			xtest = point[0]
			ytest = point[1]

			_points.each do |p|
				xcur = p[0]
				ycur = p[1]
				xlast = last_point[0]
				ylast = last_point[1]
				if ycur < ytest && ylast >= ytest || ylast < ytest && ycur >= ytest
					oddNodes = !oddNodes if xcur + (ytest - ycur) / (ylast - ycur) * (xlast - xcur) < xtest
				end
				last_point = p
			end

			oddNodes
		end
	end

	def self.logger
		@logger
	end

	def self.is_occupied(px)
		return (px.nil? or (px < 127 and px >= 0))
	end

	def self.is_free(px)
		return !is_occupied(px)
	end

	# If pixel at slice[yStart] is connected to an In Event in lInEventConnectedx
	def self.isConnectedToInEvent(lInEventConnectedx, slice, yStart)
		(yStart..slice.length-1).each do |y|
			if !is_occupied(slice[y])
				return false
			elsif is_occupied(slice[y]) and (lInEventConnectedx.include?(y) or lInEventConnectedx.include?(y+1) or lInEventConnectedx.include?(y-1))
				return true
			end
		end
		return false
	end

	# Check if this is the right most edge ob an obstacle
	def self.isOutEvent(slice_pre, slice, yStart)
		return false if yStart == slice.length-1 or is_occupied(slice_pre[yStart-1]) or is_free(slice_pre[yStart]) or is_occupied(slice[yStart-1])
		(yStart..slice.length-1).each do |y|
			if is_free(slice_pre[y]) and is_free(slice[y])
				return true
			elsif (is_occupied(slice_pre[y]) or is_occupied(slice_pre[y-1])) and is_occupied(slice[y])
				return false
			end
		end
		return true
	end

	# Check if slice[yStart] is top left edge of a new cell inside of an obstacle
	def self.isInnerEvent(slice_pre, slice, yStart)
		# Check for border from y at yStart to end of slice
		(yStart..slice.length-1).each do |y|
			if !is_occupied(slice[y]) and !is_occupied(slice_pre[y]) 
				# pixel on the left is free space => not a new cell
				return false
			end
			if ((y > 0 and is_occupied(slice_pre[y-1])) or is_occupied(slice_pre[y])) and is_occupied(slice[y])
				# reached bottom of a new inner cell
				return true
			end
		end
		# bottom of cell is end of image
		return true
	end

	def self.isFirstYinSlice(y, px_pre)
		return y == 0
	end

	def self.isLastYinSlice(yStart, slice)
		return true if yStart == slice.length-1

		(yStart..slice.length-1).each do |y|
			return false if !is_occupied(slice[y])
		end

		return true
	end

	# Implements the Boustrophedon Cellular Decomposition
	# input is map[x1...xn] = [y1..yn]
	def self.decompose(map)
		all_cells = []
		cur_cells = [Cell.new()]

		lInEventConnected = [] # Remember In Event, [x] = [y1, y2, ...]
		slice_pre = nil
		map.each_with_index do |slice, x|
			cur_cell_num = 0
			cell_in_event = nil # Cell before In event
			lInEventConnected[x] = []
			slice.each_with_index do |value, y|
				# Checks if this point is connected to an In event
				bConnectedToInEvent = false
				if isConnectedToInEvent(lInEventConnected[x-1], slice, y)
					lInEventConnected[x].push(y)
					bConnectedToInEvent = true
				end

				if (x == 0 and !isFirstYinSlice(y, slice[y-1]) and !is_occupied(slice[y-1]) and is_occupied(slice[y]) and !isLastYinSlice(y, slice)) or # In event on first column
						(x  > 0 and !isFirstYinSlice(y, slice[y-1]) and !is_occupied(slice_pre[y]) and !is_occupied(slice[y-1]) and is_occupied(slice[y]) and !bConnectedToInEvent and !isLastYinSlice(y, slice)) # Can't have In event on x == 0
					# In Event
					@logger.debug "IN #{[x, y]}"
					lInEventConnected[x].push(y)

					# Close current cell
					cell_in_event = cur_cells[cur_cell_num]
					ceil_point = cell_in_event.rm_ceil_point()
					@logger.debug "Close #{cur_cell_num} #{cell_in_event}"
					all_cells.push(cell_in_event)
					cur_cells.delete_at(cur_cell_num)

					if !isFirstYinSlice(y, slice[y-1])
						# Open new top cell
						cell_top = Cell.new([ceil_point], [[x, y-1]])
						@logger.debug "Create top cell #{cur_cell_num} #{cell_top}"
						cur_cells.insert(cur_cell_num, cell_top)
						cur_cell_num+=1
						cell_in_event.add_neighbour_end(cell_top)
					end
				elsif x > 0 and !isFirstYinSlice(y, slice[y-1]) and isOutEvent(slice_pre, slice, y) and isConnectedToInEvent(lInEventConnected[x-1], slice_pre, y)
					# Out Event => Close two current cells and create new
					@logger.debug "OUT #{[x, y]}"
					
					# Close current cell
					cell_close_top = cur_cells[cur_cell_num]
					ceil_point = cell_close_top.rm_ceil_point()
					@logger.debug "Close top cell #{cur_cell_num} #{cell_close_top}"
					all_cells.push(cell_close_top)
					cur_cells.delete_at(cur_cell_num)

					# Close next cell
					cell_close_bottom = cur_cells[cur_cell_num]
					@logger.debug "Close bottom cell #{cur_cell_num} #{cell_close_bottom}"
					all_cells.push(cell_close_bottom)
					cur_cells.delete_at(cur_cell_num)

					# Open new cell
					new_cell = Cell.new([ceil_point])
					@logger.debug "Create #{cur_cell_num} #{new_cell}"
					cur_cells.insert(cur_cell_num, new_cell)
					cell_close_top.add_neighbour_end(new_cell)
					cell_close_bottom.add_neighbour_front(new_cell)
				elsif (isFirstYinSlice(y, slice[y-1]) and !is_occupied(slice[y])) or (is_occupied(slice[y-1]) and !is_occupied(slice[y]))
					# Ceil event
					#@logger.debug "ceil #{x} #{y} to cell #{cur_cell_num} - #{cur_cells[cur_cell_num]}"
					if cell_in_event != nil
						# Create bottom cell from previous In event
						cell_bottom = Cell.new()
						cur_cells.insert(cur_cell_num, cell_bottom)
						cell_bottom.add_neighbour_front(cell_in_event)
						@logger.debug "Create bottom cell #{cur_cell_num} #{cell_bottom} at #{[x,y]}"
						cell_in_event = nil
					elsif (x > 0 and y > 1 and isInnerEvent(slice_pre, slice, y))
						# Discovered a new inner cell inside of an obstacle
						cell = Cell.new()
						cur_cells.insert(cur_cell_num, cell)
						@logger.debug "Create inner cell #{cur_cell_num} #{cell} at #{[x,y]}"
					end
					last_floor_point = cur_cells[cur_cell_num].get_last_floor_point()
					if last_floor_point and y > last_floor_point[1]
						# Ceil event is below floor point of current cell => Current cell should already have been closed
						@logger.debug "Lost cell #{cur_cell_num}"
						cell_close = cur_cells[cur_cell_num]
						@logger.debug "Close #{cur_cell_num} #{cell_close}"
						all_cells.push(cell_close)
						cur_cells.delete_at(cur_cell_num)
					end
					cur_cells[cur_cell_num].add_ceil_point([x, y])
				elsif (y > 0 and is_free(slice[y-1]) and is_occupied(slice[y])) or (is_free(slice[y]) and isLastYinSlice(y, slice))
					# Floor event
					y_add = (is_free(slice[y]) and isLastYinSlice(y, slice)) ? y : y-1
					#@logger.debug "floor #{x} #{y_add} to cell #{cur_cell_num} - #{cur_cells[cur_cell_num]}"
					cur_cells[cur_cell_num].add_floor_point([x, y_add])
					cur_cell_num+=1
				end
			end
			slice_pre = slice
		end
		all_cells += cur_cells
		return all_cells
	end

	# Recursively follows start_cells list of neighbours and appends them to reachable_cells
	def self.follow_neighours(reachable_cells, start_cell)
		start_cell.neighbours.each do |cell|
			if !reachable_cells.include? cell
				reachable_cells.push(cell)
				follow_neighours(reachable_cells, cell)
			end
		end
	end

	# Executes the Boustrophedon Cellular Decomposition
	# and optionally filteres for only from the start_pos reachable cells
	def self.run(map, start_pos=nil)
		cells = decompose(map)
		if start_pos != nil
			# filter reachable cells from start position
			start_cell = nil
			# locate start cell
			cells.each do |cell|
				if cell.contains? start_pos
					start_cell = cell
					break
				end
			end
			reachable_cells = [start_cell]
			follow_neighours(reachable_cells, start_cell)
			return reachable_cells
		end
		return cells
	end
end

if __FILE__ == $0
	BoustrophedonDecomposition.logger.level = Logger::INFO

	indata = ARGF.read
	begin
		map = JSON.parse(indata)
		BoustrophedonDecomposition.logger.info "Boustrophedon_decomposition started."
		cells = BoustrophedonDecomposition.decompose(map)
		BoustrophedonDecomposition.logger.info "boustrophedon_decomposition done."

		puts cells.select{ |cell| cell.points.length > 2}.map{ |cell| cell.points }.to_json
	rescue => e
		BoustrophedonDecomposition.logger.error e.backtrace
		tmpfile = Tempfile.new('boustrophedon_request')
		BoustrophedonDecomposition.logger.error "boustrophedon_decomposition failed, saving map to #{tmpfile.path}"
		tmpfile.write(indata)
		ObjectSpace.undefine_finalizer(tmpfile) # do not remove the file
		tmpfile.close

		#binding.pry # open console
	end
end
