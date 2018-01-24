import { Component, OnInit } from '@angular/core';

@Component({
  selector: 'app-stdpage',
  templateUrl: './stdpage.component.html',
  styleUrls: ['./stdpage.component.css']
})
export class StdpageComponent implements OnInit {

  content = "This is a standard page.";

  constructor() { }

  ngOnInit() {
  }

}
