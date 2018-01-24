import { async, ComponentFixture, TestBed } from '@angular/core/testing';

import { StdpageComponent } from './stdpage.component';

describe('StdpageComponent', () => {
  let component: StdpageComponent;
  let fixture: ComponentFixture<StdpageComponent>;

  beforeEach(async(() => {
    TestBed.configureTestingModule({
      declarations: [ StdpageComponent ]
    })
    .compileComponents();
  }));

  beforeEach(() => {
    fixture = TestBed.createComponent(StdpageComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
